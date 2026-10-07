/* Interrupt handlers. Everything not listed here falls to the startup
 * file's Default_Handler (an infinite loop). */
#include "stm32f4xx_hal.h"
#include "board.h"

extern UART_HandleTypeDef huart_bridge;

void SysTick_Handler(void)          { HAL_IncTick(); }
void BRIDGE_USART_IRQHandler(void)  { HAL_UART_IRQHandler(&huart_bridge); }
void BRIDGE_DMA_RX_IRQHandler(void) { HAL_DMA_IRQHandler(huart_bridge.hdmarx); }
void BRIDGE_DMA_TX_IRQHandler(void) { HAL_DMA_IRQHandler(huart_bridge.hdmatx); }
