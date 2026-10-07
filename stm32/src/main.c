/**
 * main.c  –  STM32F405 pose receiver for the ESP32-S3 WiFi-UART bridge.
 *
 *  Forward: ESP32 ──UART RX (DMA, circular)──► parser ──► latest pose
 *  Reverse: status frame every 20 ms ──UART TX (DMA)──► ESP32 ──UDP──► laptop
 *
 * The parser re-syncs on the 0xABCD magic and checks each frame's CRC.
 * If no valid pose arrives for FAILSAFE_TIMEOUT_MS, the receiver enters
 * failsafe (where a flight controller would hold position or land).
 */
#include "stm32f4xx_hal.h"
#include "board.h"
#include "pose_rx.h"

#define RX_BUF_SIZE 1024u          /* ~11 ms of line time at 921600 baud     */

UART_HandleTypeDef huart_bridge;
static DMA_HandleTypeDef hdma_rx, hdma_tx;

static uint8_t  rx_buf[RX_BUF_SIZE];
static uint32_t rx_tail;
static volatile uint8_t rx_restart;

static uint8_t  tx_frame[REV_FRAME_SIZE];
static uint32_t tx_counter;

static void Error_Handler(void)
{
    __disable_irq();
    for (;;) {}
}

/* ── Clock: 168 MHz from HSI or HSE ─────────────────────────────────────── */

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#if USE_HSE
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = HSE_VALUE / 1000000u;    /* 1 MHz PLL input */
#else
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;                 /* 1 MHz PLL input */
#endif
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLN     = 336;
    osc.PLL.PLLP     = RCC_PLLP_DIV2;             /* 168 MHz SYSCLK */
    osc.PLL.PLLQ     = 7;                         /* 48 MHz USB     */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;           /* 42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;           /* 84 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ── UART + DMA ─────────────────────────────────────────────────────────── */

static void uart_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BRIDGE_GPIO_CLK_EN();
    BRIDGE_USART_CLK_EN();
    BRIDGE_DMA_CLK_EN();

    gpio.Pin       = BRIDGE_TX_PIN | BRIDGE_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;                 /* idle-high if ESP32 is off */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BRIDGE_GPIO_AF;
    HAL_GPIO_Init(BRIDGE_GPIO_PORT, &gpio);

    hdma_rx.Instance                 = BRIDGE_DMA_RX_STREAM;
    hdma_rx.Init.Channel             = BRIDGE_DMA_RX_CHANNEL;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart_bridge, hdmarx, hdma_rx);

    hdma_tx.Instance                 = BRIDGE_DMA_TX_STREAM;
    hdma_tx.Init.Channel             = BRIDGE_DMA_TX_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart_bridge, hdmatx, hdma_tx);

    huart_bridge.Instance          = BRIDGE_USART;
    huart_bridge.Init.BaudRate     = BRIDGE_BAUD;
    huart_bridge.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_bridge.Init.StopBits     = UART_STOPBITS_1;
    huart_bridge.Init.Parity       = UART_PARITY_NONE;
    huart_bridge.Init.Mode         = UART_MODE_TX_RX;
    huart_bridge.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_bridge.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart_bridge) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(BRIDGE_DMA_RX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BRIDGE_DMA_RX_IRQn);
    HAL_NVIC_SetPriority(BRIDGE_DMA_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(BRIDGE_DMA_TX_IRQn);
    HAL_NVIC_SetPriority(BRIDGE_USART_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BRIDGE_USART_IRQn);
}

static void rx_start(void)
{
    /* A DMA error on TX also lands in the error callback while RX is still
     * running; stop RX first so the restart can't fail as busy. */
    if (huart_bridge.RxState != HAL_UART_STATE_READY)
        HAL_UART_AbortReceive(&huart_bridge);
    rx_tail = 0;
    pose_rx_reset_parser();
    if (HAL_UART_Receive_DMA(&huart_bridge, rx_buf, RX_BUF_SIZE) != HAL_OK)
        Error_Handler();
}

/* The HAL stops RX DMA on any line error; restart it from the main loop. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart_bridge) {
        pose_rx_count_uart_error();
        rx_restart = 1;
    }
}

/* ── RX polling + status ────────────────────────────────────────────────── */

static void rx_poll(uint32_t now)
{
    uint32_t head = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart_bridge.hdmarx);
    if (head == RX_BUF_SIZE) head = 0;
    while (rx_tail != head) {
        pose_rx_feed(rx_buf[rx_tail], now);
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    }
}

static void status_send(uint32_t now)
{
    pose_rx_build_status(tx_frame, tx_counter, now);
    if (HAL_UART_Transmit_DMA(&huart_bridge, tx_frame, REV_FRAME_SIZE) == HAL_OK)
        tx_counter++;
}

/* ── LED ────────────────────────────────────────────────────────────────── */

static void led_init(void)
{
#if HAS_LED
    GPIO_InitTypeDef gpio = {0};
    LED_GPIO_CLK_EN();
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &gpio);
#endif
}

static void led_update(uint32_t now, int failsafe)
{
#if HAS_LED
    /* Solid when poses arrive, 5 Hz blink in failsafe. */
    int on = failsafe ? ((now / 100) & 1) : 1;
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN,
                      (on ^ LED_ACTIVE_LOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)now; (void)failsafe;
#endif
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void)
{
    HAL_Init();
    clock_init();
    led_init();
    uart_init();
    rx_start();

    uint32_t next_status = HAL_GetTick();

    for (;;) {
        if (rx_restart) {
            rx_restart = 0;
            rx_start();
        }
        uint32_t now = HAL_GetTick();
        rx_poll(now);
        int failsafe = pose_rx_tick(now);

        /* Skip a slot rather than queue if the previous frame is still going. */
        if ((int32_t)(now - next_status) >= 0) {
            next_status = now + STATUS_PERIOD_MS;
            if (huart_bridge.gState == HAL_UART_STATE_READY)
                status_send(now);
        }

        led_update(now, failsafe);
    }
}
