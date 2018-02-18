#include "robot/hw/motor_driver_hw.h"

#include <assert.h>

#define MOTOR_DRIVER_MOTOR_1_INA_GPIO GPIOE
#define MOTOR_DRIVER_MOTOR_1_INA_GPIO_PIN GPIO_PIN_4
#define MOTOR_DRIVER_MOTOR_1_INA_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_1_INB_GPIO GPIOE
#define MOTOR_DRIVER_MOTOR_1_INB_GPIO_PIN GPIO_PIN_5
#define MOTOR_DRIVER_MOTOR_1_INB_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_1_PWM_GPIO GPIOB
#define MOTOR_DRIVER_MOTOR_1_PWM_GPIO_PIN GPIO_PIN_10
#define MOTOR_DRIVER_MOTOR_1_PWM_GPIO_AF GPIO_AF1_TIM2
#define MOTOR_DRIVER_MOTOR_1_PWM_GPIO_CLK() __HAL_RCC_GPIOB_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_1_PWM TIM2
#define MOTOR_DRIVER_MOTOR_1_PWM_CH TIM_CHANNEL_3
#define MOTOR_DRIVER_MOTOR_1_PWM_CLK() __HAL_RCC_TIM2_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_2_INA_GPIO GPIOE
#define MOTOR_DRIVER_MOTOR_2_INA_GPIO_PIN GPIO_PIN_6
#define MOTOR_DRIVER_MOTOR_2_INA_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_2_INB_GPIO GPIOE
#define MOTOR_DRIVER_MOTOR_2_INB_GPIO_PIN GPIO_PIN_3
#define MOTOR_DRIVER_MOTOR_2_INB_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_2_PWM_GPIO GPIOB
#define MOTOR_DRIVER_MOTOR_2_PWM_GPIO_PIN GPIO_PIN_11
#define MOTOR_DRIVER_MOTOR_2_PWM_GPIO_AF GPIO_AF1_TIM2
#define MOTOR_DRIVER_MOTOR_2_PWM_GPIO_CLK() __HAL_RCC_GPIOB_CLK_ENABLE()

#define MOTOR_DRIVER_MOTOR_2_PWM TIM2
#define MOTOR_DRIVER_MOTOR_2_PWM_CH TIM_CHANNEL_4
#define MOTOR_DRIVER_MOTOR_2_PWM_CLK() __HAL_RCC_TIM2_CLK_ENABLE()

// setting to ultrasonic freq to minimize noise
#define MOTOR_DRIVER_MOTOR_PWM_HZ 20000

uint16_t pwm_counts = 1;

typedef enum MotorDriverDirection {
  MotorDriverDirection_Brake,
  MotorDriverDirection_Reverse,
  MotorDriverDirection_Forward
} MotorDriverDirection;

void MotorDriverHw_Config(MotorDriverHw* p_this);

void MotorDriverHw_ConfigPwmFreq(TIM_HandleTypeDef* tim, uint32_t period_freq);
void MotorDriverHw_SetDirection(MotorDriverHw* p_this,
                                MotorDriverDirection motor_driver_direction);
void MotorDriverHw_SetPower(MotorDriverHw* p_this, float abs_percent);

void MotorDriverHw_ConfigPwmFreq(TIM_HandleTypeDef* tim, uint32_t period_freq) {
  pwm_counts = (uint32_t)((SystemCoreClock / 2) / (period_freq)) - 1;

  tim->Init.Prescaler = 0;
  tim->Init.Period = pwm_counts;
}

void MotorDriverHw_SetDirection(MotorDriverHw* p_this,
                                MotorDriverDirection motor_driver_direction) {
  switch (motor_driver_direction) {
    case MotorDriverDirection_Brake:
      HAL_GPIO_WritePin(p_this->ina_gpiotype, p_this->ina_pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(p_this->inb_gpiotype, p_this->inb_pin, GPIO_PIN_SET);
      break;
    case MotorDriverDirection_Reverse:
      HAL_GPIO_WritePin(p_this->ina_gpiotype, p_this->ina_pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(p_this->inb_gpiotype, p_this->inb_pin, GPIO_PIN_RESET);
      break;
    case MotorDriverDirection_Forward:
      HAL_GPIO_WritePin(p_this->ina_gpiotype, p_this->ina_pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(p_this->inb_gpiotype, p_this->inb_pin, GPIO_PIN_SET);
      break;
  }
}

void MotorDriverHw_SetPower(MotorDriverHw* p_this, float abs_percent) {
  uint16_t ccr_count = (uint16_t)((float)pwm_counts * abs_percent);
  __HAL_TIM_SET_COMPARE(&p_this->pwm_handle, p_this->pwm_channel, ccr_count);
}

void MotorDriverHw_SetPowerAndDirection(MotorDriverHw* p_this, float percent) {
  if (percent == 0) {
    MotorDriverHw_SetDirection(p_this, MotorDriverDirection_Brake);
    MotorDriverHw_SetPower(p_this, 0.0);
  } else if (percent < 0) {
    MotorDriverHw_SetDirection(p_this, MotorDriverDirection_Reverse);
    MotorDriverHw_SetPower(p_this, -percent);
  } else {
    MotorDriverHw_SetDirection(p_this, MotorDriverDirection_Forward);
    MotorDriverHw_SetPower(p_this, percent);
  }
}

void MotorDriverHw_Config(MotorDriverHw* p_this);

void MotorDriverHw_Init(MotorDriverHw* p_this, MotorTypes motor_type) {
  p_this->type = motor_type;
  MotorDriverHw_Config(p_this);
  MotorDriverHw_SetDirection(p_this, 0.0);
}

void MotorDriverHw_Config(MotorDriverHw* p_this) {
  // pwm: pull down on board
  // inx: floating

  GPIO_InitTypeDef inx_gpio;
  inx_gpio.Mode = GPIO_MODE_OUTPUT_PP;
  inx_gpio.Pull = GPIO_PULLUP;
  inx_gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;

  GPIO_InitTypeDef pwm_gpio;
  pwm_gpio.Mode = GPIO_MODE_AF_PP;
  pwm_gpio.Pull = GPIO_NOPULL;
  pwm_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  TIM_HandleTypeDef pwm;
  pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  pwm.Init.ClockDivision = 0;
  pwm.Init.RepetitionCounter = 0;
  pwm.State = HAL_TIM_STATE_RESET;
  MotorDriverHw_ConfigPwmFreq(&pwm, MOTOR_DRIVER_MOTOR_PWM_HZ);

  TIM_OC_InitTypeDef pwm_oc;
  pwm_oc.OCMode = TIM_OCMODE_PWM2;
  pwm_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  pwm_oc.OCFastMode = TIM_OCFAST_DISABLE;
  pwm_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm_oc.OCPolarity = TIM_OCPOLARITY_LOW;
  pwm_oc.Pulse = 0;

  switch (p_this->type) {
    case MotorTypes_Motor1:
      MOTOR_DRIVER_MOTOR_1_INA_GPIO_CLK();
      p_this->ina_gpiotype = MOTOR_DRIVER_MOTOR_1_INA_GPIO;
      p_this->ina_pin = MOTOR_DRIVER_MOTOR_1_INA_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_1_INB_GPIO_CLK();
      p_this->inb_gpiotype = MOTOR_DRIVER_MOTOR_1_INB_GPIO;
      p_this->inb_pin = MOTOR_DRIVER_MOTOR_1_INB_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_1_PWM_GPIO_CLK();
      pwm_gpio.Alternate = MOTOR_DRIVER_MOTOR_1_PWM_GPIO_AF;
      p_this->pwm_gpiotype = MOTOR_DRIVER_MOTOR_1_PWM_GPIO;
      p_this->pwm_pin = MOTOR_DRIVER_MOTOR_1_PWM_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_1_PWM_CLK();
      pwm.Instance = MOTOR_DRIVER_MOTOR_1_PWM;

      p_this->pwm_handle = pwm;
      p_this->pwm_channel = MOTOR_DRIVER_MOTOR_1_PWM_CH;
      break;
    case MotorTypes_Motor2:
      MOTOR_DRIVER_MOTOR_2_INA_GPIO_CLK();
      p_this->ina_gpiotype = MOTOR_DRIVER_MOTOR_2_INA_GPIO;
      p_this->ina_pin = MOTOR_DRIVER_MOTOR_2_INA_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_2_INB_GPIO_CLK();
      p_this->inb_gpiotype = MOTOR_DRIVER_MOTOR_2_INB_GPIO;
      p_this->inb_pin = MOTOR_DRIVER_MOTOR_2_INB_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_2_PWM_GPIO_CLK();
      pwm_gpio.Alternate = MOTOR_DRIVER_MOTOR_2_PWM_GPIO_AF;
      p_this->pwm_gpiotype = MOTOR_DRIVER_MOTOR_2_PWM_GPIO;
      p_this->pwm_pin = MOTOR_DRIVER_MOTOR_2_PWM_GPIO_PIN;

      MOTOR_DRIVER_MOTOR_2_PWM_CLK();
      pwm.Instance = MOTOR_DRIVER_MOTOR_2_PWM;

      p_this->pwm_handle = pwm;
      p_this->pwm_channel = MOTOR_DRIVER_MOTOR_2_PWM_CH;
      break;
  }

  inx_gpio.Pin = p_this->ina_pin;
  HAL_GPIO_Init(p_this->ina_gpiotype, &inx_gpio);

  inx_gpio.Pin = p_this->inb_pin;
  HAL_GPIO_Init(p_this->inb_gpiotype, &inx_gpio);

  pwm_gpio.Pin = p_this->pwm_pin;
  HAL_GPIO_Init(p_this->pwm_gpiotype, &pwm_gpio);

  HAL_TIM_PWM_Init(&p_this->pwm_handle);
  HAL_TIM_PWM_ConfigChannel(&p_this->pwm_handle, &pwm_oc, p_this->pwm_channel);
  HAL_TIM_PWM_Start(&p_this->pwm_handle, p_this->pwm_channel);
}
