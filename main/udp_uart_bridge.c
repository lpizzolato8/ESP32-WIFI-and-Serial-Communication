/**
 * udp_uart_bridge.c  –  Bidirectional UDP↔UART bridge
 *
 * Forward  (Task 1 – bridge_task):
 *   Laptop UDP → ESP32 port 4444 → UART TX (GPIO17) → laptop serial RX
 *
 * Reverse  (Task 2 – uart_udp_task):
 *   Laptop serial TX → UART RX (GPIO18) → ESP32 UDP → laptop port 4445
 *   Laptop IP is learned from the first forward UDP packet received.
 *
 * UART pinout:
 *   GPIO17 → CP2102 RX  (UART TX, forward path)
 *   GPIO18 ← CP2102 TX  (UART RX, reverse path)
 */

#include "udp_uart_bridge.h"

#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

/* ── Configuration ──────────────────────────────────────────────────────── */

#define BRIDGE_UDP_PORT   4444
#define REVERSE_UDP_PORT  4445  /* laptop listens here for reverse data */
#define REV_FRAME_SIZE    1034  /* must match Python REV_FRAME_SIZE     */

#define UART_PORT_NUM    UART_NUM_1
#define UART_BAUD_RATE   921600
#define UART_TX_PIN      17       /* GPIO17 → CP2102 RX (forward TX)         */
#define UART_RX_PIN      18       /* GPIO18 ← CP2102 TX (reverse RX)         */
#define UART_BUF_SIZE    4096     /* 2× REV_FRAME_SIZE to handle scan + leftover    */

#define TASK_STACK       4096
#define TASK_PRIORITY    6        /* Higher than default; keeps latency low  */
#define REV_TASK_STACK   6144
#define REV_TASK_PRIO    5

/* ── Internals ──────────────────────────────────────────────────────────── */

static const char *TAG = "udp_uart";

/* Laptop address learned from the first forward UDP packet. */
static volatile bool      s_client_valid = false;
static struct sockaddr_in s_client_addr  = {0};

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM,
                                        UART_BUF_SIZE * 2,  /* RX ring buf */
                                        UART_BUF_SIZE * 2,  /* TX ring buf */
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                                 UART_TX_PIN,
                                 UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,   /* RTS – unused */
                                 UART_PIN_NO_CHANGE)); /* CTS – unused */

    ESP_LOGI(TAG, "UART%d ready  baud=%d  TX=GPIO%d  RX=GPIO%d",
             UART_PORT_NUM, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
}

static void bridge_task(void *arg)
{
    
    ESP_LOGI(TAG, "bridge_task started");
    /* ── Open UDP socket ────────────────────────────────────────────────── */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(BRIDGE_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP bridge listening on 192.168.4.1:%d", BRIDGE_UDP_PORT);

    /* ── Receive loop ───────────────────────────────────────────────────── */
    static uint8_t rx_buf[UART_BUF_SIZE];   /* static: keeps off the stack */

    while (1) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);

        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&src, &src_len);

        if (len < 0) {
 	    ESP_LOGW(TAG, "recvfrom() error: errno %d", errno);
            continue;
        }
        if (len == 0) {
            continue;
        }

        /* Learn the laptop's address for the reverse UDP path. */
        if (!s_client_valid) {
            s_client_addr       = src;
            s_client_addr.sin_port = htons(REVERSE_UDP_PORT);
            s_client_valid      = true;
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
            ESP_LOGI(TAG, "Learned client address: %s → reverse UDP to :%d", ip, REVERSE_UDP_PORT);
        }

        /* Send a 1-byte UDP ack back to the sender *before* writing to UART.
         * The laptop timestamps this to split WiFi latency from UART latency. */
        uint8_t ack = 0xAC;
        sendto(sock, &ack, 1, 0, (struct sockaddr *)&src, src_len);

        /* Write raw bytes to UART – non-blocking from caller's perspective
         * because the UART driver has its own TX ring buffer.              */

        int written = uart_write_bytes(UART_PORT_NUM, rx_buf, len);

        if (written != len) {
            ESP_LOGW(TAG, "UART TX short write: wanted %d got %d", len, written);
        }

        
         
        char src_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &src.sin_addr, src_ip, sizeof(src_ip));
        ESP_LOGD(TAG, "[%s] %d bytes → UART", src_ip, len);
        
    }

    close(sock);
    vTaskDelete(NULL);
}

/* ── Reverse task: UART RX → UDP TX ─────────────────────────────────────── */

static const uint8_t REV_MAGIC[4] = {'R', 'E', 'V', 0xAA};

static void uart_udp_task(void *arg)
{
    /* 2× buffer so a frame spanning two reads always fits alongside leftovers. */
    static uint8_t buf[REV_FRAME_SIZE * 2];
    int filled = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "reverse udp socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Reverse UDP task started, waiting for client address");

    while (1) {
        if (!s_client_valid) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int len = uart_read_bytes(UART_PORT_NUM, buf + filled,
                                  sizeof(buf) - filled, pdMS_TO_TICKS(50));
        if (len <= 0) continue;
        filled += len;

        /* Scan forward until buf starts with the magic bytes, discarding
         * any partial/misaligned data from joining mid-frame.             */
        while (filled >= 4 &&
               memcmp(buf, REV_MAGIC, 4) != 0) {
            memmove(buf, buf + 1, --filled);
        }

        if (filled < REV_FRAME_SIZE) continue;

        int sent = sendto(sock, buf, REV_FRAME_SIZE, 0,
                          (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
        if (sent < 0) {
            ESP_LOGW(TAG, "Reverse UDP sendto failed: errno %d", errno);
        } else {
            ESP_LOGD(TAG, "Reverse: %d bytes UART → UDP", sent);
        }

        /* Shift any leftover bytes (start of next frame) to the front. */
        filled -= REV_FRAME_SIZE;
        if (filled > 0) memmove(buf, buf + REV_FRAME_SIZE, filled);
    }

    close(sock);
    vTaskDelete(NULL);
}

void udp_uart_bridge_start(void)
{
    uart_init();
    xTaskCreate(bridge_task,   "udp_uart", TASK_STACK,     NULL, TASK_PRIORITY, NULL);
    xTaskCreate(uart_udp_task, "uart_udp", REV_TASK_STACK, NULL, REV_TASK_PRIO, NULL);
}
