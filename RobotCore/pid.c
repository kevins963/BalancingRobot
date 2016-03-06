/******************/
/* system include */
/******************/

/*****************/
/* local include */
/*****************/
#include "pid.h"
/******************/
/* private define */
/******************/

/*******************/
/* private typedef */
/*******************/

/****************/
/* private enum */
/****************/

/*********************/
/* private structure */
/*********************/

/*****************/
/* private const */
/*****************/

/*********************/
/* private variables */
/*********************/

/*******************************/
/* private function definition */
/*******************************/

/********************************/
/* public functions declaration */
/********************************/

void Pid_Init( sPidData * pPidData, float kp, float ki, float kd, float limitMax, float limitMin )
{
    pPidData->kp = kp;
    pPidData->ki = ki;
    pPidData->kd = kd;

    pPidData->a0 = kp + ki + kd;
    pPidData->a1 = ( -pPidData->kp ) - ( 2.0f * pPidData->kd );
    pPidData->a2 = pPidData->kd;

    pPidData->prev2InErr = 0;
    pPidData->prevInErr = 0;
    pPidData->prevOut = 0;

    pPidData->limitMax = limitMax;
    pPidData->limitMin = limitMin;
}

float Pid_Run( sPidData * pPidData, float inputError )
{
    //
    float out;

    /* out = (kp + ki + kd) * inErr +
             ( -kp - 2*kd ) inErr1
             kd * inErr2
             + out1
             Simplified expanding out1
             (kp + ki + kd) * inErr +
             ( -kp - 2*kd ) inErr1
             kd * inErr2 +
             (kp + ki + kd) * inErr1 +
             ( -kp - 2*kd ) inErr2
             kd * inErr3 */
    out = ( pPidData->a0 * inputError ) + 
    ( pPidData->a1 * pPidData->prevInErr ) + 
    ( pPidData->a2 * pPidData->prev2InErr ) + 
    ( pPidData->prevOut );

    if( out > pPidData->limitMax )
    {
        out = pPidData->limitMax;
    }
    else if( out < pPidData->limitMin )
    {
        out = pPidData->limitMin;
    }

    //update hold vars
    pPidData->prev2InErr = pPidData->prevInErr;
    pPidData->prevInErr = inputError;
    pPidData->prevOut = out;

    return ( out );
}

/*********************************/
/* private functions declaration */
/*********************************/
