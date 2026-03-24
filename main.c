
/**
 * main.c  –  ESP32-S3-N16R8  |  WiFi AP + UDP→UART bridge
 *
 * Boot sequence:
 *   1. NVS flash init
 *   2. LwIP / netif init
 *   3. Default event loop
 *   4. Soft-AP  (192.168.4.1, SSID/pass in wifi_ap.c)
 *   5. UDP → UART bridge  (port 4444, UART config in udp_uart_bridge.c)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"

#include "wifi_ap.h"
#include "udp_uart_bridge.h"

static const char *TAG = "main";

void app_main(void)
{
    /* ── NVS ──────────────────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue – erasing and re-initialising");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── Network interface + event loop ───────────────────────────────── */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── WiFi soft-AP ─────────────────────────────────────────────────── */
    wifi_ap_init();

    /* ── UDP → UART bridge ────────────────────────────────────────────── */
    udp_uart_bridge_start();

    ESP_LOGI(TAG, "Ready. Send UDP datagrams to 192.168.4.1:4444");
}