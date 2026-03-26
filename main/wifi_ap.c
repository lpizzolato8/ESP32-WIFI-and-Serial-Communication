/**
 * wifi_ap.c  –  ESP32-S3 soft-AP (hotspot) initialisation
 *
 * Configurable via the defines below.  The AP is started with WPA2-PSK
 * by default; set AP_PASSWORD to "" and AP_AUTHMODE to WIFI_AUTH_OPEN for
 * an open (no-password) network.
 */

#include "wifi_ap.h"

#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"

/* ── Configuration ──────────────────────────────────────────────────────── */

#define AP_SSID        "ESP32S3-Hotspot"   /* Network name (max 31 chars)  */
#define AP_PASSWORD    "esp32pass"          /* Min 8 chars for WPA2; "" = open */
#define AP_CHANNEL     6                   /* 1-13                         */
#define AP_MAX_CLIENTS 4                   /* Simultaneous connections     */
#define AP_AUTHMODE    WIFI_AUTH_WPA2_PSK  /* Change to WIFI_AUTH_OPEN if  */
                                           /* AP_PASSWORD is ""            */

/* ── Internals ──────────────────────────────────────────────────────────── */

static const char *TAG = "wifi_ap";

static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "Soft-AP started  SSID: \"%s\"  CH: %d", AP_SSID, AP_CHANNEL);
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *e = event_data;
            ESP_LOGI(TAG, "Client connected   MAC: " MACSTR "  AID: %d",
                     MAC2STR(e->mac), e->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *e = event_data;
            ESP_LOGI(TAG, "Client disconnected  MAC: " MACSTR "  AID: %d",
                     MAC2STR(e->mac), e->aid);
            break;
        }

        default:
            break;
        }
    }
}

void wifi_ap_init(void)
{
    /* Create the default AP netif (assigns 192.168.4.1 by default) */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    /* Initialise the WiFi driver with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handler for AP events */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    /* Build the AP configuration.
     * ssid and password are uint8_t arrays – must use memcpy, not assignment. */
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.ap.ssid,     AP_SSID,     strlen(AP_SSID));
    memcpy(wifi_config.ap.password, AP_PASSWORD, strlen(AP_PASSWORD));
    wifi_config.ap.ssid_len       = (uint8_t)strlen(AP_SSID);
    wifi_config.ap.channel        = AP_CHANNEL;
    wifi_config.ap.max_connection = AP_MAX_CLIENTS;
    wifi_config.ap.authmode       = AP_AUTHMODE;
    wifi_config.ap.pmf_cfg.required = false;

    /* Force OPEN mode when no password is supplied */
    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP ready  IP: 192.168.4.1  Password: \"%s\"", AP_PASSWORD);
}
