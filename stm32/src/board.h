#pragma once
/**
 * board.h  –  Pin and peripheral choices for the STM32F405 receiver.
 *
 * Defaults: USART1 on PA9 (TX) / PA10 (RX), which is the "TX1/RX1" pad pair
 * on most F405 flight controllers. Check your board's pinout; if UART1 is
 * taken, the commented blocks below are ready-made alternatives.
 *
 * Wiring to the ESP32-S3:
 *   ESP32 GPIO17 (TX) ──► STM32 RX   (forward: pose frames)
 *   ESP32 GPIO18 (RX) ◄── STM32 TX   (reverse: status frames)
 *   GND ─────────────── GND
 */

/* ── Clock ──────────────────────────────────────────────────────────────────
 * 0 = internal 16 MHz HSI (works on any board, ~1% accuracy, fine for UART).
 * 1 = external crystal; set HSE_VALUE in stm32f4xx_hal_conf.h (8 MHz on most
 *     flight controllers, 12 MHz on the Adafruit Feather F405).            */
#define USE_HSE                 0

/* ── Bridge UART: USART1, PA9 TX / PA10 RX, DMA2 ────────────────────────── */
#define BRIDGE_USART            USART1
#define BRIDGE_USART_CLK_EN()   __HAL_RCC_USART1_CLK_ENABLE()
#define BRIDGE_USART_IRQn       USART1_IRQn
#define BRIDGE_USART_IRQHandler USART1_IRQHandler
#define BRIDGE_GPIO_PORT        GPIOA
#define BRIDGE_GPIO_CLK_EN()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define BRIDGE_TX_PIN           GPIO_PIN_9
#define BRIDGE_RX_PIN           GPIO_PIN_10
#define BRIDGE_GPIO_AF          GPIO_AF7_USART1
#define BRIDGE_DMA_CLK_EN()     __HAL_RCC_DMA2_CLK_ENABLE()
#define BRIDGE_DMA_RX_STREAM    DMA2_Stream2
#define BRIDGE_DMA_RX_CHANNEL   DMA_CHANNEL_4
#define BRIDGE_DMA_RX_IRQn      DMA2_Stream2_IRQn
#define BRIDGE_DMA_RX_IRQHandler DMA2_Stream2_IRQHandler
#define BRIDGE_DMA_TX_STREAM    DMA2_Stream7
#define BRIDGE_DMA_TX_CHANNEL   DMA_CHANNEL_4
#define BRIDGE_DMA_TX_IRQn      DMA2_Stream7_IRQn
#define BRIDGE_DMA_TX_IRQHandler DMA2_Stream7_IRQHandler

/* USART3, PB10 TX / PB11 RX:
 *   USART3, __HAL_RCC_USART3_CLK_ENABLE(), USART3_IRQn, USART3_IRQHandler,
 *   GPIOB, __HAL_RCC_GPIOB_CLK_ENABLE(), GPIO_PIN_10, GPIO_PIN_11,
 *   GPIO_AF7_USART3, __HAL_RCC_DMA1_CLK_ENABLE(),
 *   RX: DMA1_Stream1 / DMA_CHANNEL_4, TX: DMA1_Stream3 / DMA_CHANNEL_4
 *
 * USART6, PC6 TX / PC7 RX:
 *   USART6, __HAL_RCC_USART6_CLK_ENABLE(), USART6_IRQn, USART6_IRQHandler,
 *   GPIOC, __HAL_RCC_GPIOC_CLK_ENABLE(), GPIO_PIN_6, GPIO_PIN_7,
 *   GPIO_AF8_USART6, __HAL_RCC_DMA2_CLK_ENABLE(),
 *   RX: DMA2_Stream1 / DMA_CHANNEL_5, TX: DMA2_Stream6 / DMA_CHANNEL_5
 */

#define BRIDGE_BAUD             921600

/* ── Status LED (optional) ──────────────────────────────────────────────────
 * Solid = poses arriving, fast blink = failsafe. Set HAS_LED to 0 if you
 * don't know the LED pin. Flight controllers often use PB5, PB4 or PC14,
 * usually active-low.                                                     */
#define HAS_LED                 0
#define LED_GPIO_PORT           GPIOB
#define LED_GPIO_CLK_EN()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED_PIN                 GPIO_PIN_5
#define LED_ACTIVE_LOW          1

/* ── Behaviour ──────────────────────────────────────────────────────────── */
#define STATUS_PERIOD_MS        20    /* 50 Hz reverse status frames        */
