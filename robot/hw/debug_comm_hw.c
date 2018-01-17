#include "robot/hw/debug_comm_hw.h"

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

// private function declarations
void DebugCommHw_ConfigUsart(void);

// public functions
void DebugCommHw_Init(void) {
  
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
    HAL_GPIO_Init(DEBUG_COMM_USART_TX_GPIO_PORT, &GPIO_InitStruct);
    
    uart_gpio.Pin = DEBUG_COMM_USART_RX_GPIO_PIN;
    HAL_GPIO_Init(DEBUG_COMM_USART_RX_GPIO_PORT, &GPIO_InitStruct);
    
    // Configure USART
    DEBUG_COMM_USART_CLK();
    
    UART_HandleTypeDef uart_handle;
    
      uart_handle.Instance        = DEBUG_COMM_USART;

  uart_handle.Init.BaudRate   = 115200;
  uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits   = UART_STOPBITS_1;
  uart_handle.Init.Parity     = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  uart_handle.Init.Mode       = UART_MODE_TX_RX;
  uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&uart_handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}