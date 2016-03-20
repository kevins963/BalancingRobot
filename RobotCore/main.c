/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    09-October-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mems.h"
#include "pid.h"
#include "motordriver.h"
//#include "stm32f429i_discovery_gyroscope.h"
#include <limits.h>
#include <math.h>

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef struct  
{
    uint32_t lastTick;
    uint32_t period;
} sTimer;

typedef struct orientation
{
    float pitch;
    float roll;
    float yaw;


    float pitchAcc;
    float rollAcc;
    float yawAcc;

    float gyroData[ 3 ];
    float gyroDataCalibration[ 3 ];
    int16_t accelData[3];
}
sOrientationData;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
sTimer mAccelTimer;
sTimer mLcdTimer;
sTimer mMotorTimer;
sTimer mLedTimer;
int16_t mX, mY, mZ;
float mXg, mYg, mZg;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

void Timer_Init( sTimer *pTimer, uint32_t period );
uint8_t Timer_Run( sTimer *pTimer );
sMotorDriver mMotorDriverB;


int16_t maxX, maxY, currentPos = 0;
/* Private functions ---------------------------------------------------------*/

sPidData mPidAngle;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void DrawGraph( int16_t currentValue, int16_t lastValue, int16_t min, int16_t max, uint32_t color );

void Orientation_Init( sOrientationData * data );

void Orientation_Init( sOrientationData * data )
{
    data->roll = 0;
    data->pitch = 0;
    data->yaw = 0;
}

void Orientation_Calc1( sOrientationData * data );

void Orientation_Calc1( sOrientationData * data )
{
    static float dt = 0.01f;
    
    float aX, aY, aZ;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    
    aX = data->accelData[ 0 ];
    aY = data->accelData[ 1 ];
    aZ = data->accelData[ 2 ];

    data->pitchAcc = atan2f( aY, sqrtf( aX * aX + aZ * aZ ) );
    data->rollAcc = atan2f( -aX, aZ );
    data->pitchAcc = data->pitchAcc * 180.0f / M_PI;
    data->rollAcc = data->rollAcc * 180.0f / M_PI;

    float gyroPercent = 0.98f;
    float accelPercent = 1.0f - gyroPercent;
    
    data->pitch = ( ( data->pitch + data->gyroData[ 0 ] * dt ) * gyroPercent ) + ( data->pitchAcc * accelPercent ); // Angle around the X-axis
    data->roll  = ( ( data->roll  + data->gyroData[ 1 ] * dt ) * gyroPercent ) + ( data->rollAcc  * accelPercent );
    data->yaw   = ( data->gyroData[ 2 ] ) * dt; // Angle around the X-axis

    
    
                                                               // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    //int forceMagnitudeApprox = abs( accData[ 0 ] ) + abs( accData[ 1 ] ) + abs( accData[ 2 ] );
    //if( forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768 )
    //{
    //    // Turning around the X axis results in a vector on the Y-axis
    //    pitchAcc = atan2f( ( float )accData[ 1 ], ( float )accData[ 2 ] ) * 180 / M_PI;
    //    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    //    // Turning around the Y axis results in a vector on the X-axis
    //    rollAcc = atan2f( ( float )accData[ 0 ], ( float )accData[ 2 ] ) * 180 / M_PI;
    //    *roll = *roll * 0.98 + rollAcc * 0.02;
    //}
}

void Orientation_Calc2( sOrientationData * data );

sOrientationData mOrientationData;
int main(void)
{

    /* STM32F4xx HAL library initialization:
        - Configure the Flash prefetch, Flash preread and Buffer caches
        - Systick timer is configured by default as source of time base, but user 
                can eventually implement his proper time base source (a general purpose 
                timer for example or other time source), keeping in mind that Time base 
                duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
                handled in milliseconds basis.
        - Low Level Initialization
        */
    HAL_Init();

    /* Configure the System clock to 180 MHz */
    SystemClock_Config();


    /* Add your application code here
        */

    //Config BSP
    BSP_JOY_Init();
    BSP_LED_Init( LED_GREEN );
    BSP_LED_Init( LED_BLUE );
    BSP_LED_Init( LED_RED );

    while( HAL_GetTick() < 3000 )
    { }

    Mems_Init();
    MotorDriver_Init( &mMotorDriverB );

    /*##-2- Touch screen initialization ########################################*/
    //Touchscreen_Calibration();
    //BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

    /* Infinite loop */
  
    int lastTick = 0;
    int lastTick2 = 0;
    int x = 0;
    int y = 0;
    unsigned int color = 0xffffbb33;
    unsigned int rgb;
    char string[16];


    Timer_Init( &mAccelTimer, 10 );
    Timer_Init( &mLcdTimer, 100 );
    Timer_Init( &mMotorTimer, 1 );
    Timer_Init( &mLedTimer, 100 );

    Orientation_Init( &mOrientationData );

    Pid_Init( &mPidAngle, 1000.0F / 120.0F, 0, 0, 1000, -1000 );

    while( 1 )
    {
      if( Timer_Run( &mLedTimer ) )
      {
        static int counterLed = 0;
        
        counterLed++;
        
        if( counterLed % 2 == 0 )
        {
            BSP_LED_Toggle( LED_GREEN );
        }
        if( counterLed % 4 == 0 )
        {
            BSP_LED_Toggle( LED_RED );
        }
        if( counterLed % 8 == 0 )
        {
            BSP_LED_Toggle( LED_BLUE );
        }
        
        
      }
    }
    /*
    while( 1 )
    {
        static int counter = 0;
        static float gyroCalibration[ 3 ] = { 0,0,0 };
        static float gyroBuf[ 3 ];

    //calibrating
        if( Timer_Run( &mAccelTimer ) )
        {

            //BSP_GYRO_GetXYZ( gyroBuf );

            gyroCalibration[ 0 ] += gyroBuf[ 0 ];
            gyroCalibration[ 1 ] += gyroBuf[ 1 ];
            gyroCalibration[ 2 ] += gyroBuf[ 2 ];

            counter++;
        }

        if( counter == 20 )
        {
            mOrientationData.gyroDataCalibration[ 0 ] = ( gyroCalibration[ 0 ] / 20.0f );
            mOrientationData.gyroDataCalibration[ 1 ] = ( gyroCalibration[ 1 ] / 20.0f );
            mOrientationData.gyroDataCalibration[ 2 ] = ( gyroCalibration[ 2 ] / 20.0f );
            break;
        }
    }

    while (1)
    {
        static int16_t motorSpeed = 500;
        static int16_t motorDir = -1;

        if( Timer_Run( &mAccelTimer ) )
        {
            static int16_t buf[ 3 ];
            static float gyroBuf[ 3 ];
            static float lastOut, currentOut;
            Mems_GetXYZ(buf);
            BSP_GYRO_GetXYZ( gyroBuf );
            //DrawGraph( buf[ 1 ], mY, LCD_COLOR_GREEN );
            //DrawGraph( buf[ 2 ], mZ, LCD_COLOR_BLUE );

            mX = buf[ 0 ];
            mY = buf[ 1 ];
            mZ = buf[ 2 ]; 
            
            mXg = gyroBuf[ 0 ];
            mYg = gyroBuf[ 1 ];
            mZg = gyroBuf[ 2 ];

            mOrientationData.accelData[ 0 ] = ( buf[ 0 ] );
            mOrientationData.accelData[ 1 ] = ( buf[ 1 ] );
            mOrientationData.accelData[ 2 ] = ( buf[ 2 ] );
            
            mOrientationData.gyroData[ 0 ] = ( gyroBuf[ 0 ] - mOrientationData.gyroDataCalibration[ 0 ] );
            mOrientationData.gyroData[ 1 ] = ( gyroBuf[ 1 ] - mOrientationData.gyroDataCalibration[ 1 ] );
            mOrientationData.gyroData[ 2 ] = ( gyroBuf[ 2 ] - mOrientationData.gyroDataCalibration[ 2 ] );

            float lastRoll = mOrientationData.roll;
            float lastRollAcc = mOrientationData.rollAcc;
            
            Orientation_Calc1( &mOrientationData );

            lastOut = currentOut;
            currentOut = Pid_Run( &mPidAngle, 0 - mOrientationData.roll );
            
            if( mOrientationData.roll > 120 || 
                mOrientationData.roll < -120 )
            {
                MotorDriver_SetSpeed( &mMotorDriverB, 0 );
            }
            else
            {
                MotorDriver_SetSpeed( &mMotorDriverB, currentOut );
            }
            
            DrawGraph( INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, LCD_COLOR_BLACK );
            //DrawGraph( (int16_t)(mOrientationData.roll * 10), (int16_t)(lastRoll * 10), -1800, 1800, LCD_COLOR_RED );
            //DrawGraph( ( int16_t )( mOrientationData.rollAcc * 10 ), ( int16_t )( lastRollAcc * 10 ), -1800, 1800, LCD_COLOR_BLUE );
            DrawGraph( ( int16_t )( currentOut / 2 ),(int16_t)( lastOut/2 ), mPidAngle.limitMin, mPidAngle.limitMax, LCD_COLOR_GREEN );
            
            currentPos++;
            
            DrawGraph( INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, LCD_COLOR_GREEN );
            //sprintf( string, "%d", mZ );
            //BSP_LCD_ClearStringLine( 3 );
            //BSP_LCD_DisplayStringAtLine( 3, string );
        }

        if( Timer_Run( &mLcdTimer ) )
        {

//            BSP_LCD_SetTextColor( LCD_COLOR_RED ); 
//            ////color+=0x030201;
//            sprintf( string, "%d", (int16_t) mOrientationData.pitch );
//            BSP_LCD_ClearStringLine( 1 );
//            BSP_LCD_DisplayStringAtLine( 1, string );
//
//            sprintf( string, "%d", ( int16_t ) mOrientationData.roll );
//            BSP_LCD_ClearStringLine( 2 );
//            BSP_LCD_DisplayStringAtLine( 2, string );
//
//            sprintf( string, "%d", ( int16_t ) mOrientationData.yaw );
//            BSP_LCD_ClearStringLine( 3 );
//            BSP_LCD_DisplayStringAtLine( 3, string );
//
//            sprintf( string, "%d", ( int16_t )mOrientationData.pitchAcc );
//            BSP_LCD_ClearStringLine( 4 );
//            BSP_LCD_DisplayStringAtLine( 4, string );
//
//            sprintf( string, "%d", ( int16_t )mOrientationData.rollAcc );
//            BSP_LCD_ClearStringLine( 5 );
//            BSP_LCD_DisplayStringAtLine( 5, string );

            //sprintf( string, "%d", (int)mXg );
            //BSP_LCD_ClearStringLine( 4 );
            //BSP_LCD_DisplayStringAtLine( 4, string );

            //sprintf( string, "%d", (int)mYg );
            //BSP_LCD_ClearStringLine( 5 );
            //BSP_LCD_DisplayStringAtLine( 5, string );

            //sprintf( string, "%d", (int)mZg );
            //BSP_LCD_ClearStringLine( 6 );
            //BSP_LCD_DisplayStringAtLine( 6, string );
        }
    }
    */
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void Timer_Init( sTimer *pTimer, uint32_t period )
{
    pTimer->lastTick = 0;
    pTimer->period = period;
}

uint8_t Timer_Run( sTimer *pTimer )
{
    uint8_t ret = 0;
    uint32_t curTick = HAL_GetTick();

    if( HAL_GetTick() > pTimer->lastTick + pTimer->period )
    {
        ret = 1;
        pTimer->lastTick = curTick;
    }

    return ret;
}

/* COMPASS / ACCELERO IO functions */
void    COMPASSACCELERO_IO_Init(void)
{
}
void    COMPASSACCELERO_IO_ITConfig(void)
{
}
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
}
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
  return 0;
}


/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
