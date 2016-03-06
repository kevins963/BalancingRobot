#include "MotorDriver.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "system_stm32f4xx.h"
//#include "stm32f429xx.h"
#include "main.h"

GPIO_InitTypeDef  GPIO_InitStruct;
TIM_HandleTypeDef timerHandle;
TIM_OC_InitTypeDef ocHandle;

void HwMotorDriver_Init( sMotorDriver * thisClass );
void HwMotorDriver_SetSpeed( sMotorDriverIndvidual * pSuperClass, int16_t speed );

void MotorDriver_Init( sMotorDriver * thisClass )
{
    HwMotorDriver_Init( thisClass );
}

void HwMotorDriver_Init( sMotorDriver * thisClass )
{


    //Setup timers
    //Setup INa/INb outputs

    //4khz
    
    uint32_t dutyCycleCount = 1000;
    uint32_t period = 4000;

    uint32_t prescaler = ( uint32_t )( ( SystemCoreClock ) / ( dutyCycleCount * period ) ) - 1;

    //Period
    timerHandle.Instance = TIM1;

    timerHandle.Init.Period = dutyCycleCount;
    timerHandle.Init.Prescaler = prescaler;
    timerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timerHandle.Init.ClockDivision = 0;
    timerHandle.Init.RepetitionCounter = 0;
    timerHandle.State = HAL_TIM_STATE_RESET;

    if( HAL_TIM_PWM_Init( &timerHandle ) != HAL_OK )
    {
        Error_Handler();
    }

    //Configure channel
    ocHandle.OCMode = TIM_OCMODE_PWM2;
    ocHandle.OCIdleState = TIM_OCIDLESTATE_RESET;
    ocHandle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    ocHandle.OCFastMode = TIM_OCFAST_DISABLE;
    ocHandle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ocHandle.OCPolarity = TIM_OCPOLARITY_LOW;
    ocHandle.Pulse = 500;

    if( HAL_TIM_PWM_ConfigChannel( &timerHandle, &ocHandle, TIM_CHANNEL_3 ) != HAL_OK )
    {
        /* Configuration Error */
        Error_Handler();
    }

    if( HAL_TIM_PWM_ConfigChannel( &timerHandle, &ocHandle, TIM_CHANNEL_2 ) != HAL_OK )
    {
        /* Configuration Error */
        Error_Handler();
    }
    
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* -2- Configure PB11 and PB4 IOs in output push-pull mode to drive external
    LEDs */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init( GPIOE, &GPIO_InitStruct );

    thisClass->motorDriverLeft.gpioPortA = GPIOE;
    thisClass->motorDriverLeft.gpioPinA = GPIO_PIN_2;
    thisClass->motorDriverLeft.gpioPortB = GPIOE;
    thisClass->motorDriverLeft.gpioPinB = GPIO_PIN_3;
    thisClass->motorDriverLeft.channel = TIM_CHANNEL_2;

    thisClass->motorDriverLeft.gpioPortA = GPIOE;
    thisClass->motorDriverLeft.gpioPinA = GPIO_PIN_4;
    thisClass->motorDriverLeft.gpioPortB = GPIOE;
    thisClass->motorDriverLeft.gpioPinB = GPIO_PIN_5;
    thisClass->motorDriverLeft.channel = TIM_CHANNEL_3;

    //TESTING
    
    
    
}

void HAL_TIM_PWM_MspInit( TIM_HandleTypeDef *htim )
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Enable GPIO Channels Clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure (PA.08, TIM1_CH1), (PB.13, TIM1_CH1N), (PA.09, TIM1_CH2),
    (PB.14, TIM1_CH2N), (PA.10, TIM1_CH3), (PB.15, TIM1_CH3N),
    (PA.11, TIM1_CH4) in push-pull, alternate function mode  */

    /* Common configuration for all channels */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    /* GPIO TIM1_Channel2 configuration */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    /* GPIO TIM1_Channel3 configuration */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );
}

void MotorDriver_SetSpeed( sMotorDriver * thisClass, int16_t speed )
{
    HwMotorDriver_SetSpeed( &thisClass->motorDriverLeft, speed );
    HwMotorDriver_SetSpeed( &thisClass->motorDriverRight, speed );
}

void HwMotorDriver_SetSpeed( sMotorDriverIndvidual * pSuperClass, int16_t speed )
{
    uint16_t absSpeed;

    absSpeed = speed < 0 ? -speed : speed;

    if( absSpeed > 1000 )
    {
        absSpeed = 1000;
    }

    if( speed < 0 )
    {
        HAL_GPIO_WritePin( pSuperClass->gpioPortA, pSuperClass->gpioPinA, GPIO_PIN_SET );
        HAL_GPIO_WritePin( pSuperClass->gpioPortB, pSuperClass->gpioPinB, GPIO_PIN_RESET );
    }
    else
    {
        HAL_GPIO_WritePin( pSuperClass->gpioPortA, pSuperClass->gpioPinA, GPIO_PIN_RESET );
        HAL_GPIO_WritePin( pSuperClass->gpioPortB, pSuperClass->gpioPinB, GPIO_PIN_SET );
    }

    ocHandle.Pulse = absSpeed;

    if( HAL_TIM_PWM_ConfigChannel( &timerHandle, &ocHandle, pSuperClass->channel ) != HAL_OK )
    {
        /* Configuration Error */
        //Error_Handler();
    }

    if( HAL_TIM_PWM_Start( &timerHandle, pSuperClass->channel ) != HAL_OK )
    {
        /* Starting Error */
        //Error_Handler();
    }

}


