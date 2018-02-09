#include "robot/hw/encoder_hw.h"

#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

#define ENCODER_MOTOR_1_ENCA_GPIO GPIOC
#define ENCODER_MOTOR_1_ENCA_GPIO_PIN GPIO_PIN_6
#define ENCODER_MOTOR_1_ENCA_GPIO_AF GPIO_AF3_TIM8
#define ENCODER_MOTOR_1_ENCA_GPIO_CLK() __HAL_RCC_GPIOC_CLK_ENABLE()

#define ENCODER_MOTOR_1_ENCB_GPIO GPIOC
#define ENCODER_MOTOR_1_ENCB_GPIO_PIN GPIO_PIN_7
#define ENCODER_MOTOR_1_ENCB_GPIO_AF GPIO_AF3_TIM8
#define ENCODER_MOTOR_1_ENCB_GPIO_CLK() __HAL_RCC_GPIOC_CLK_ENABLE()

#define ENCODER_MOTOR_1_TIM TIM8
#define ENCODER_MOTOR_1_TIM_CLK() __HAL_RCC_TIM8_CLK_ENABLE()

#define ENCODER_MOTOR_2_ENCA_GPIO GPIOE
#define ENCODER_MOTOR_2_ENCA_GPIO_PIN GPIO_PIN_11
#define ENCODER_MOTOR_2_ENCA_GPIO_AF GPIO_AF1_TIM1
#define ENCODER_MOTOR_2_ENCA_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define ENCODER_MOTOR_2_ENCB_GPIO GPIOE
#define ENCODER_MOTOR_2_ENCB_GPIO_PIN GPIO_PIN_9
#define ENCODER_MOTOR_2_ENCB_GPIO_AF GPIO_AF1_TIM1
#define ENCODER_MOTOR_2_ENCB_GPIO_CLK() __HAL_RCC_GPIOE_CLK_ENABLE()

#define ENCODER_MOTOR_2_TIM TIM1
#define ENCODER_MOTOR_2_TIM_CLK() __HAL_RCC_TIM1_CLK_ENABLE()

TIM_HandleTypeDef handle_motor_1_;
TIM_HandleTypeDef handle_motor_2_;

uint16_t EncoderHw_Motor1Count(Encoder* p_this) {
  return __HAL_TIM_GET_COUNTER(&handle_motor_1_);
}

uint16_t EncoderHw_Motor2Count(Encoder* p_this) {
  return __HAL_TIM_GET_COUNTER(&handle_motor_2_);
}

void EncoderHw_Config(EncoderHw* p_this) {
  // configure pins
  GPIO_InitTypeDef tim_gpio;

  tim_gpio.Mode = GPIO_MODE_AF_PP;
  tim_gpio.Pull = GPIO_PULLUP;
  tim_gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  TIM_HandleTypeDef tim;
  tim.Init.Prescaler = 0;
  tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim.Init.Period = 0xFFFF;

  TIM_Encoder_InitTypeDef tim_encoder;
  tim_encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  tim_encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  tim_encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  tim_encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  tim_encoder.IC1Filter = 0;
  tim_encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  tim_encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  tim_encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  tim_encoder.IC2Filter = 0;

  switch (p_this->motor_type) {
    case MotorTypes_Motor1:
      ENCODER_MOTOR_1_ENCA_GPIO_CLK();
      tim_gpio.Alternate = ENCODER_MOTOR_1_ENCA_GPIO_AF;
      tim_gpio.Pin = ENCODER_MOTOR_1_ENCA_GPIO_PIN;
      HAL_GPIO_Init(ENCODER_MOTOR_1_ENCA_GPIO, &tim_gpio);

      ENCODER_MOTOR_1_ENCB_GPIO_CLK();
      tim_gpio.Alternate = ENCODER_MOTOR_1_ENCB_GPIO_AF;
      tim_gpio.Pin = ENCODER_MOTOR_1_ENCB_GPIO_PIN;
      HAL_GPIO_Init(ENCODER_MOTOR_1_ENCB_GPIO, &tim_gpio);

      ENCODER_MOTOR_1_TIM_CLK();
      tim.Instance = ENCODER_MOTOR_1_TIM;
      HAL_TIM_Encoder_Init(&tim, &tim_encoder);
      HAL_TIM_Encoder_Start(&tim, TIM_CHANNEL_ALL);

      p_this->base.v_functions.GetEncoderCount = EncoderHw_Motor1Count;
      handle_motor_1_ = tim;
      break;
    case MotorTypes_Motor2:
      ENCODER_MOTOR_2_ENCA_GPIO_CLK();
      tim_gpio.Alternate = ENCODER_MOTOR_2_ENCA_GPIO_AF;
      tim_gpio.Pin = ENCODER_MOTOR_2_ENCA_GPIO_PIN;
      HAL_GPIO_Init(ENCODER_MOTOR_2_ENCA_GPIO, &tim_gpio);

      ENCODER_MOTOR_2_ENCB_GPIO_CLK();
      tim_gpio.Alternate = ENCODER_MOTOR_2_ENCB_GPIO_AF;
      tim_gpio.Pin = ENCODER_MOTOR_2_ENCB_GPIO_PIN;
      HAL_GPIO_Init(ENCODER_MOTOR_2_ENCB_GPIO, &tim_gpio);

      ENCODER_MOTOR_2_TIM_CLK();
      tim.Instance = ENCODER_MOTOR_2_TIM;
      HAL_TIM_Encoder_Init(&tim, &tim_encoder);
      HAL_TIM_Encoder_Start(&tim, TIM_CHANNEL_ALL);

      p_this->base.v_functions.GetEncoderCount = EncoderHw_Motor2Count;
      handle_motor_2_ = tim;
      break;
  }
}

void EncoderHw_Init(EncoderHw* p_this, MotorTypes motor_type) {
  InitDerivedClass(EncoderHw, Encoder);
  p_this->motor_type = motor_type;

  EncoderHw_Config(p_this);
}
