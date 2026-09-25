#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Start the bidirectional bridge (two background FreeRTOS tasks).
 *
 * Forward: UDP port 4444 → UART TX. Reverse: UART RX → UDP port 4445.
 * Forward datagrams are written verbatim to UART (no parsing).
 *
 * Call after wifi_ap_init().
 */
void udp_uart_bridge_start(void);

#ifdef __cplusplus
}
#endif