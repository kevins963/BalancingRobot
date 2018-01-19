#include "robot/hw/debug_comm_hw.h"

#include "robot/core/circle_buffer.h"
#include "robot/hw/stm32f4xx_it.h"
#include "robot/hw/system_config.h"
#include "robot/hw/system_handler.h"
#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

// private defines

#define DEBUG_COMM_USART_TX_GPIO_PORT GPIOD
#define DEBUG_COMM_USART_TX_GPIO_PIN GPIO_PIN_8

#define DEBUG_COMM_USART_RX_GPIO_PORT GPIOD
#define DEBUG_COMM_USART_RX_GPIO_PIN GPIO_PIN_9

#define DEBUG_COMM_USART_GPIO_CLK() __HAL_RCC_GPIOD_CLK_ENABLE()
#define DEBUG_COMM_USART_GPIO_AF GPIO_AF7_USART3

#define DEBUG_COMM_USART USART3
#define DEBUG_COMM_USART_CLK() __HAL_RCC_USART3_CLK_ENABLE()

// private structs
UART_HandleTypeDef uart_handle;
CircleBuffer rx_;
CircleBuffer tx_;

// private function declarations
void DebugCommHw_ConfigUsart(void);

// public functions
void DebugCommHw_Init(void) {
  DebugCommHw_ConfigUsart();
  HAL_UART_Transmit_IT(&uart_handle, buf, 20);
}

// private functions
void DebugCommHw_ConfigUsart(void) {
  // Config GPIO to usart
  GPIO_InitTypeDef uart_gpio;
  uart_gpio.Mode = GPIO_MODE_AF_PP;
  uart_gpio.Pull = GPIO_PULLUP;
  uart_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  uart_gpio.Alternate = DEBUG_COMM_USART_GPIO_AF;

  DEBUG_COMM_USART_GPIO_CLK();

  uart_gpio.Pin = DEBUG_COMM_USART_TX_GPIO_PIN;
  HAL_GPIO_Init(DEBUG_COMM_USART_TX_GPIO_PORT, &uart_gpio);

  uart_gpio.Pin = DEBUG_COMM_USART_RX_GPIO_PIN;
  HAL_GPIO_Init(DEBUG_COMM_USART_RX_GPIO_PORT, &uart_gpio);

  // Configure USART
  DEBUG_COMM_USART_CLK();

  HAL_NVIC_SetPriority(USART3_IRQn, NVIC_PRIORITY_DEBUG_COMM_USART, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  uart_handle.Instance = DEBUG_COMM_USART;

  uart_handle.Init.BaudRate = 115200;
  uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits = UART_STOPBITS_1;
  uart_handle.Init.Parity = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart_handle.Init.Mode = UART_MODE_TX_RX;
  uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&uart_handle) != HAL_OK) {
    InitErrorHandler();
  }
}

void USART3_IRQHandler() {
  UART_HandleTypeDef* huart = &uart_handle;
  uint32_t isrflags = READ_REG(huart->Instance->SR);
  uint32_t cr1its = READ_REG(huart->Instance->CR1);
  uint32_t cr3its = READ_REG(huart->Instance->CR3);
  uint32_t errorflags = 0x00U;
  uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE |
                                      USART_SR_NE));
  if (errorflags == RESET) {
    /* UART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_SR_RXNE) != RESET) &&
        ((cr1its & USART_CR1_RXNEIE) != RESET)) {
      CircleBuffer_WriteByte(&rx_, huart->Instance->DR & 0x00ff);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != RESET) &&
      (((cr3its & USART_CR3_EIE) != RESET) ||
       ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET))) {
    /* UART parity error interrupt occurred ----------------------------------*/
    if (((isrflags & USART_SR_PE) != RESET) &&
        ((cr1its & USART_CR1_PEIE) != RESET)) {
      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART noise error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_NE) != RESET) &&
        ((cr3its & USART_CR3_EIE) != RESET)) {
      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART frame error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_FE) != RESET) &&
        ((cr3its & USART_CR3_EIE) != RESET)) {
      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART Over-Run interrupt occurred --------------------------------------*/
    if (((isrflags & USART_SR_ORE) != RESET) &&
        ((cr3its & USART_CR3_EIE) != RESET)) {
      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* UART in mode Transmitter
     * ------------------------------------------------*/
    if (((isrflags & USART_SR_TXE) != RESET) &&
        ((cr1its & USART_CR1_TXEIE) != RESET)) {
      CircleBuffer_WriteByte(&rx_, huart->Instance->DR & 0x00ff);
      return;
    }

    /* UART in mode Transmitter end
     * --------------------------------------------*/
    if (((isrflags & USART_SR_TC) != RESET) &&
        ((cr1its & USART_CR1_TCIE) != RESET)) {
      UART_EndTransmit_IT(huart);
      return;
    }
  }
