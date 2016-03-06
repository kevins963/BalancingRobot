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
    float kp;
    float ki;
    float kd;

    float a0;
    float a1;
    float a2;

    float prevOut;
    float prevInErr;
    float prev2InErr;

    float limitMax;
    float limitMin;
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

void Pid_Init( sPidData * pPidData, float kp, float ki, float kd, float limitMax, float limitMin );

float Pid_Run( sPidData * pPidData, float inputError );
#endif /* pid_h__ */
