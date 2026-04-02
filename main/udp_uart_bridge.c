/**
 * udp_uart_bridge.c  –  UDP datagram → UART passthrough
 *
 * Laptop sends a binary frame as a UDP packet to 192.168.4.1:BRIDGE_UDP_PORT.
 * The ESP32 writes the raw bytes to UART (TX pin) verbatim.
 * No parsing, no framing — pure passthrough.
 *
 * UART pinout (change UART_TX_PIN / UART_RX_PIN to match your wiring):
 *   GPIO17 → STM32 RX
 *   GPIO18 ← STM32 TX  (wired but unused in this one-way flow)
 *
 * Tune BRIDGE_UDP_PORT, UART_BAUD_RATE, and UART_BUF_SIZE below.
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

#define BRIDGE_UDP_PORT  4444

#define UART_PORT_NUM    UART_NUM_1
#define UART_BAUD_RATE   921600
#define UART_TX_PIN      17       /* GPIO17 → CP2102 RX → USB serial         */
#define UART_RX_PIN      18       /* GPIO18 ← CP2102 TX (unused)             */
#define UART_BUF_SIZE    1024     /* Must be > largest expected UDP payload  */

#define TASK_STACK       4096
#define TASK_PRIORITY    6        /* Higher than default; keeps latency low  */

/* ── Internals ──────────────────────────────────────────────────────────── */

static const char *TAG = "udp_uart";

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

void udp_uart_bridge_start(void)
{
    uart_init();
    xTaskCreate(bridge_task, "udp_uart", TASK_STACK, NULL, TASK_PRIORITY, NULL);
}
