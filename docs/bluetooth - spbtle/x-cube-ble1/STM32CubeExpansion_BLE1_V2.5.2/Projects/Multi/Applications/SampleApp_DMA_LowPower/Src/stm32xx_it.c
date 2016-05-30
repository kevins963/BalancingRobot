/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32xx_it.h"
#include "main.h"
#include "app_service.h"
#include "debug.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup DMA_LP_App
 *  @{
 */
 
/** @defgroup INTERRUPT_HANDLER
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t ms_counter = 0;
volatile int button_event = 0;
/* SPI handler declared in "main.c" file */
extern SPI_HandleTypeDef SpiHandle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  NMI_Handler This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  HardFault_Handler This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  SVC_Handler This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  DebugMon_Handler This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  PendSV_Handler This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  SysTick_Handler This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  
  ms_counter++;
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void RTC_WAKEUP_IRQHandler(void)
{
  TIMER_RTC_Wakeup_Handler();
}

/**
  * @brief  BNRG_SPI_EXTI_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void BNRG_SPI_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
}

/**
  * @brief  This function handles External lines interrupt request.
  * @param  None
  * @retval None
  */
void PUSH_BUTTON_EXTI_IRQHandler(void)
{
  button_event = 1;
  SetUserProcessRequest(TRUE);
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);  
}

/**
  * @brief  This function handles DMA Rx interrupt request.  
  * @param  None
  * @retval None    
  */
#ifdef STM32L053xx
void DMA1_Channel2_3_IRQHandler(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmarx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmarx, DMA_FLAG_TC2))
  {
    BlueNRG_DMA_RxCallback();
  }
  else if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmatx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmatx, DMA_FLAG_TC3))
  {
    BlueNRG_DMA_TxCallback();
  }
}
#endif /* STM32L053xx */

#ifdef STM32L476xx
void DMA1_Channel2_IRQHandler(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmarx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmarx, DMA_FLAG_TC2))
  {
    BlueNRG_DMA_RxCallback();
  }
}

void DMA1_Channel3_IRQHandler(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmatx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmatx, DMA_FLAG_TC3))
  {
    BlueNRG_DMA_TxCallback();
  }
}
#endif /* STM32L476xx */

#ifdef STM32F401xE
void DMA2_Stream0_IRQHandler(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmarx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmarx, DMA_FLAG_TCIF0_4))
  {
    BlueNRG_DMA_RxCallback();
  }
}

void DMA2_Stream3_IRQHandler(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(SpiHandle.hdmatx, DMA_IT_TC)  && __HAL_DMA_GET_FLAG(SpiHandle.hdmatx, DMA_FLAG_TCIF3_7))
  {
    BlueNRG_DMA_TxCallback();
  }
}
#endif /* STM32F401xE */

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*
void PPP_IRQHandler(void)
{    
}
*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
