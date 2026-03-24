#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Start the UDP → UART bridge in a background FreeRTOS task.
 *
 * Listens on BRIDGE_UDP_PORT (default 4444).
 * Every UDP datagram received is written verbatim to UART (no parsing).
 *
 * Call after wifi_ap_init().
 */
void udp_uart_bridge_start(void);

#ifdef __cplusplus
}
#endif