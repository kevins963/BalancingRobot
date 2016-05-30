#ifndef pid_h__
#define pid_h__

/******************/
/* system include */
/******************/

/*****************/
/* local include */
/*****************/

/*****************/
/* public define */
/*****************/

/******************/
/* public typedef */
/******************/

/***************/
/* public enum */
/***************/

/********************/
/* public structure */
/********************/

typedef struct  
{
	float sampleRate;
	float kiDefault;
	float kdDefault;

    float kp;
    float ki;
    float kd;

	float prevError;
	float intAccError;

    float outLimit;
    float integralAccLimit;
}
sPidData;
/****************/
/* public const */
/****************/

/********************/
/* public variables */
/********************/

/******************************/
/* public function definition */
/******************************/

void Pid_Init( sPidData * pPidData, float kp, float ki, float kd, float sampleRate, float outLimit, float integralAccLimit);

float Pid_Run( sPidData * pPidData, float inputError );

float Pid_Bound(float val, float upperLimit, float lowerLimit);
#endif /* pid_h__ */
