#include "robot/hw/system_time_hw.h"

#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

void SystemTimeHw_Init(SystemTimeHw* p_this) {
  SystemTimeFunctions functions;
  functions.GetMsecTick = HAL_GetTick;
  InitDerivedClass(SystemTimeHw, SystemTime, functions);
}
