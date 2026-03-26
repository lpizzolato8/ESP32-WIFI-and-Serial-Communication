#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the WiFi soft-AP.
 *
 * Call once from app_main after nvs_flash_init() and esp_netif_init().
 * Blocks until the AP is ready to accept client connections.
 */
void wifi_ap_init(void);

#ifdef __cplusplus
}
#endif
