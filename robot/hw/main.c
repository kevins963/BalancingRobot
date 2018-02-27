#include <stdlib.h>
#include <string.h>

#include "robot/core/core.h"
#include "robot/hw/debug_comm_hw.h"
#include "robot/hw/encoder_hw.h"
#include "robot/hw/mems_hw.h"
#include "robot/hw/motor_driver_hw.h"
#include "robot/hw/system_handler.h"
#include "robot/hw/system_time_hw.h"
#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

void ConfigSystemClock(void);
void InitErrorHandler(void);

EncoderHw encoder_hw_motor_1;
EncoderHw encoder_hw_motor_2;
MotorDriverHw motor_driver_hw_motor_1;
MotorDriverHw motor_driver_hw_motor_2;
DebugCommHw debug_comm_hw;
SystemTimeHw system_time_hw;
MemsHw mems_hw;

int main() {
  HAL_Init();

  // 16 level of preemptive ISRs
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  ConfigSystemClock();

  SystemTimeHw_Init(&system_time_hw);
  DebugCommHw_Init(&debug_comm_hw);
  EncoderHw_Init(&encoder_hw_motor_1, MotorTypes_Motor1);
  EncoderHw_Init(&encoder_hw_motor_2, MotorTypes_Motor2);
  MotorDriverHw_Init(&motor_driver_hw_motor_1, MotorTypes_Motor1);
  MotorDriverHw_Init(&motor_driver_hw_motor_2, MotorTypes_Motor2);
  MemsHw_Init(&mems_hw);

  CoreAppDrivers drivers = kEmptyCoreAppDrivers;
  drivers.debug_comm = &debug_comm_hw.base;
  drivers.encoder_m1 = &encoder_hw_motor_1.base;
  drivers.encoder_m2 = &encoder_hw_motor_2.base;
  drivers.system_time = &system_time_hw.base;
  drivers.gyro = &mems_hw.gyro_driver;
  drivers.accel = &mems_hw.accel_driver;

  CoreApp_Init(&drivers);

  while (1) {
    CoreApp_Run();
  }

  return 0;
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
void ConfigSystemClock(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device
     is clocked below the maximum system frequency, to update the voltage
     scaling value regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    InitErrorHandler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    /* Initialization Error */
    InitErrorHandler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    /* Initialization Error */
    InitErrorHandler();
  }
}

void InitErrorHandler(void) {
  while (1) {
  }
}
