#ifndef PATH_FILE_H
#define PATH_FILE_H

#include "robot/core/defs.h"
#include "robot/core/integer.h"
#include "robot/core/util.h"
#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

BEGIN_STRUCT(MotorDriverHw)
MotorTypes type;
GPIO_TypeDef* ina_gpiotype;
uint32_t ina_pin;
GPIO_TypeDef* inb_gpiotype;
uint32_t inb_pin;
GPIO_TypeDef* pwm_gpiotype;
uint32_t pwm_pin;
TIM_HandleTypeDef pwm_handle;
uint32_t pwm_channel;
END_STRUCT(MotorDriverHw)

void MotorDriverHw_Init(MotorDriverHw* p_this, MotorTypes motor_type);
void MotorDriverHw_SetPowerAndDirection(MotorDriverHw* p_this, float percent);

#endif  // PATH_FILE_H
