/**
 * udp_uart_bridge.c  –  Bidirectional UDP↔UART bridge
 *
 * Forward  (Task 1 – bridge_task):
 *   Laptop UDP → ESP32 port 4444 → UART TX (GPIO17) → laptop serial RX
 *
 * Reverse  (Task 2 – uart_udp_task):
 *   Laptop serial TX → UART RX (GPIO18) → ESP32 UDP → laptop port 4445
 *   Laptop IP is learned from forward UDP packets and updated if it changes.
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
/* Frames the ESP32 relays from UART to UDP, found by their 4-byte magic and
 * sent whole, one datagram each. Sizes must match test_bridge.py and
 * stm32/src/pose_rx.h.                                                   */
#define REV_FRAME_SIZE    1034  /* "REV\xAA": laptop serial load test    */
#define STM_FRAME_SIZE      67  /* "STM\xAA": STM32 status               */
#define ECHO_FRAME_SIZE     27  /* "ECH\xAA": STM32 echo of a pose       */

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

/* Laptop address learned from forward UDP packets. Written by bridge_task,
 * read by uart_udp_task; s_client_lock keeps the struct copy consistent. */
static portMUX_TYPE       s_client_lock  = portMUX_INITIALIZER_UNLOCKED;
static bool               s_client_valid = false;
static struct sockaddr_in s_client_addr  = {0};

/* Reverse-path diagnostics, logged every STATS_PERIOD_MS by uart_udp_task.
 * UART error counts come from the driver's event queue (uart_evt_task). */
#define STATS_PERIOD_MS  5000
#define UART_EVT_QUEUE   64

static QueueHandle_t s_uart_q;
static volatile uint32_t s_fifo_ovf, s_buf_full, s_frame_err, s_parity_err, s_brk;
static uint32_t s_rx_bytes, s_skipped, s_frames_sent, s_send_fail, s_reads, s_max_burst;

static void uart_evt_task(void *arg)
{
    uart_event_t ev;
    while (1) {
        if (!xQueueReceive(s_uart_q, &ev, portMAX_DELAY)) continue;
        switch (ev.type) {
        case UART_FIFO_OVF:    s_fifo_ovf++;   break;
        case UART_BUFFER_FULL: s_buf_full++;   break;
        case UART_FRAME_ERR:   s_frame_err++;  break;
        case UART_PARITY_ERR:  s_parity_err++; break;
        case UART_BREAK:       s_brk++;        break;
        default: break;   /* UART_DATA etc: just drain the queue */
        }
    }
}

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
                                        UART_EVT_QUEUE, &s_uart_q,
                                        ESP_INTR_FLAG_IRAM));
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

        /* Learn the laptop's address for the reverse UDP path, and follow it
         * if the laptop reconnects with a different IP. */
        bool changed = false;
        taskENTER_CRITICAL(&s_client_lock);
        if (!s_client_valid ||
            s_client_addr.sin_addr.s_addr != src.sin_addr.s_addr) {
            s_client_addr          = src;
            s_client_addr.sin_port = htons(REVERSE_UDP_PORT);
            s_client_valid         = true;
            changed                = true;
        }
        taskEXIT_CRITICAL(&s_client_lock);
        if (changed) {
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

static const struct { uint8_t magic[4]; int len; } REV_FRAMES[] = {
    { {'R', 'E', 'V', 0xAA}, REV_FRAME_SIZE  },
    { {'S', 'T', 'M', 0xAA}, STM_FRAME_SIZE  },
    { {'E', 'C', 'H', 0xAA}, ECHO_FRAME_SIZE },
};

/* Length of the frame starting at p (needs 4 bytes), or 0 if no magic matches. */
static int frame_len_at(const uint8_t *p)
{
    for (size_t i = 0; i < sizeof(REV_FRAMES) / sizeof(REV_FRAMES[0]); i++) {
        if (memcmp(p, REV_FRAMES[i].magic, 4) == 0) return REV_FRAMES[i].len;
    }
    return 0;
}

static void uart_udp_task(void *arg)
{
    /* After each pass fewer than REV_FRAME_SIZE (the longest frame) bytes are
     * left, so this always has room for the next read. */
    static uint8_t buf[REV_FRAME_SIZE * 2];
    int filled = 0;
    TickType_t next_stats = xTaskGetTickCount() + pdMS_TO_TICKS(STATS_PERIOD_MS);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "reverse udp socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Reverse UDP task started, waiting for client address");

    while (1) {
        struct sockaddr_in dest;
        bool valid;
        taskENTER_CRITICAL(&s_client_lock);
        valid = s_client_valid;
        dest  = s_client_addr;
        taskEXIT_CRITICAL(&s_client_lock);

        if (!valid) {
            /* Nobody to send to yet: keep draining so the STM32's status
             * stream can't fill the RX ring and overflow the FIFO. */
            uart_flush_input(UART_PORT_NUM);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Wake on the first byte, then take whatever else is already buffered,
         * so a short echo frame isn't held up waiting for a full buffer. */
        int len = uart_read_bytes(UART_PORT_NUM, buf + filled, 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            size_t avail = 0;
            uart_get_buffered_data_len(UART_PORT_NUM, &avail);
            size_t room = sizeof(buf) - filled - len;
            if (avail > room) avail = room;
            if (avail > 0) {
                len += uart_read_bytes(UART_PORT_NUM, buf + filled + len, avail, 0);
            }
        }
        if (xTaskGetTickCount() >= next_stats) {
            next_stats += pdMS_TO_TICKS(STATS_PERIOD_MS);
            ESP_LOGI(TAG, "rev: rx=%u skipped=%u sent=%u sendfail=%u reads=%u maxburst=%u | "
                          "uart ovf=%u buffull=%u frame=%u parity=%u brk=%u",
                     (unsigned)s_rx_bytes, (unsigned)s_skipped, (unsigned)s_frames_sent,
                     (unsigned)s_send_fail, (unsigned)s_reads, (unsigned)s_max_burst,
                     (unsigned)s_fifo_ovf, (unsigned)s_buf_full, (unsigned)s_frame_err,
                     (unsigned)s_parity_err, (unsigned)s_brk);
        }
        if (len <= 0) continue;
        filled += len;
        s_rx_bytes += len;
        s_reads++;
        uint32_t burst = 0;

        /* Send every complete frame in the buffer, not just the first. */
        while (1) {
            /* Find the first magic and discard everything before it
             * (partial/misaligned data from joining mid-frame). If there is
             * no magic, keep the last 3 bytes in case one starts there.   */
            int start = 0, flen = 0;
            while (start + 4 <= filled &&
                   (flen = frame_len_at(buf + start)) == 0) {
                start++;
            }
            if (start > 0) {
                s_skipped += start;
                filled -= start;
                memmove(buf, buf + start, filled);
            }

            if (flen == 0 || filled < flen) break;

            int sent = sendto(sock, buf, flen, 0,
                              (struct sockaddr *)&dest, sizeof(dest));
            burst++;
            if (burst > s_max_burst) s_max_burst = burst;
            if (sent < 0) {
                s_send_fail++;
                ESP_LOGW(TAG, "Reverse UDP sendto failed: errno %d", errno);
            } else {
                s_frames_sent++;
                ESP_LOGD(TAG, "Reverse: %d bytes UART → UDP", sent);
            }

            /* Shift any leftover bytes (start of next frame) to the front. */
            filled -= flen;
            if (filled > 0) memmove(buf, buf + flen, filled);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

/* uart_driver_install() allocates the UART interrupt on the core it runs on.
 * app_main and the WiFi driver are on core 0, so install from a task pinned
 * to core 1 to keep the UART ISR clear of WiFi (a delayed ISR overflows the
 * 128-byte RX FIFO: ~1.4 ms at 921600 baud). */
static void bridge_start_task(void *arg)
{
    uart_init();
    xTaskCreate(bridge_task,   "udp_uart", TASK_STACK,     NULL, TASK_PRIORITY, NULL);
    xTaskCreate(uart_udp_task, "uart_udp", REV_TASK_STACK, NULL, REV_TASK_PRIO, NULL);
    xTaskCreate(uart_evt_task, "uart_evt", 2048, NULL, TASK_PRIORITY + 1, NULL);
    vTaskDelete(NULL);
}

void udp_uart_bridge_start(void)
{
    xTaskCreatePinnedToCore(bridge_start_task, "bridge_start", 4096, NULL,
                            TASK_PRIORITY, NULL, 1);
}
