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

void Pid_Init( sPidData * pPidData, float kp, float ki, float kd, float sampleRate, float outLimit, float integralAccLimit )
{
	pPidData->sampleRate = sampleRate;
	pPidData->kiDefault = ki;
	pPidData->kdDefault = kd;

	pPidData->kp = kp;
	pPidData->ki = pPidData->kiDefault / sampleRate;
	pPidData->kd = pPidData->kdDefault / sampleRate;

	pPidData->prevError = 0;
	pPidData->intAccError = 0;

    pPidData->outLimit = outLimit;
    pPidData->integralAccLimit = integralAccLimit;
}

float Pid_Run( sPidData * pPidData, float inputError )
{
    //
    float out;
	float diffError;

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
             kd * inErr3 + out2 */
	pPidData->intAccError = Pid_Bound(pPidData->intAccError + inputError, pPidData->integralAccLimit, -pPidData->integralAccLimit);
	
	diffError = inputError - pPidData->prevError;

	out = pPidData->kp * inputError +
		pPidData->ki * pPidData->intAccError +
		pPidData->kd * diffError;

	out = Pid_Bound(out, pPidData->outLimit, -pPidData->outLimit);

    //update hold vars
    pPidData->prevError = inputError;

    return ( out );
}

float Pid_Bound(float val, float upperLimit, float lowerLimit)
{
	if (val > upperLimit)
	{
		return upperLimit;
	}
	else if (val < lowerLimit)
	{
		return lowerLimit;
	}

	return val;
}


/*********************************/
/* private functions declaration */
/*********************************/
