#ifndef _H_MEMS_
#define _H_MEMS_

#include "stdint.h"

//#include "stm32f429i_discovery.h"

#include "bsp/components/lsm303d/lsm303d.h"

typedef enum
{
    ACCELERO_OK = 0,
    ACCELERO_ERROR = 1,
    ACCELERO_TIMEOUT = 2
} ACCELERO_StatusTypeDef;

uint8_t Mems_Init( void );
void    Mems_Reset( void );
void    Mems_ITConfig( void );
void    Mems_GetXYZ( int16_t *pDataXYZ );


#endif
