#ifndef _H_MOTORDRIVER__
#define _H_MOTORDRIVER__

#include "stdint.h"
#include "stm32f429xx.h"

typedef struct sMotorDriverFunctions sMotorDriverFunctions;
typedef struct sMotorDriverIndvidual sMotorDriverIndvidual;
typedef struct sMotorDriver sMotorDriver;

typedef enum
{
    eMotorDriverDirection_BrakeHigh,
    eMotorDriverDirection_BrakeLow,
    eMotorDriverDirection_Clockwise,
    eMotorDriverDirection_CounterClockwise
}
eMotorDriverDirection;

//struct sMotorDriverFunctions
//{
//    
//};

struct sMotorDriverIndvidual
{
    //sMotorDriverFunctions functions;
    GPIO_TypeDef * gpioPortA;
    uint16_t gpioPinA;
    GPIO_TypeDef * gpioPortB;
    uint16_t gpioPinB;
    uint32_t channel;
};

struct sMotorDriver
{
    sMotorDriverIndvidual motorDriverLeft;
    sMotorDriverIndvidual motorDriverRight;
};

void MotorDriver_Init( sMotorDriver * thisClass );
void MotorDriver_SetSpeed( sMotorDriver * thisClass, int16_t speed );

#endif
