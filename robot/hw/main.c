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
#include <stdint.h>
#include "l3gd20h.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

#define ACCEL_TIMER_MSEC ( 10 )

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

    int16_t pitchWhole;
    int16_t rollWhole;
    int16_t yawWhole;
    
    float pitchAccDeg;
    float rollAccDeg;
    float yawAccDeg;


    
    float pitchAccPi;
    float rollAccPi;
    float yawAccPi;

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

GYRO_DrvTypeDef *mpGyroDriver = &L3GD20HDrv;

int16_t maxX, maxY, currentPos = 0;
/* Private functions ---------------------------------------------------------*/

sPidData mPidAngle;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void BSP_INIT_GYRO( void );
void BSP_GYRO_GetXYZ(float * gyroBuf);

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
    static float dt = (float)ACCEL_TIMER_MSEC / 1000.0f;
    
    float aX, aY, aZ;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    
    aX = data->accelData[ 0 ];
    aY = data->accelData[ 1 ];
    aZ = data->accelData[ 2 ];

    data->pitchAccPi = atan2f( aY, sqrtf( aX * aX + aZ * aZ ) );
    data->rollAccPi = atan2f( aX, sqrtf( aY * aY + aZ * aZ ) );
    data->pitchAccDeg = data->pitchAccPi * 180.0f / M_PI;
    data->rollAccDeg = data->rollAccPi * 180.0f / M_PI;

    float gyroPercent = 0.98f;
    float accelPercent = 1.0f - gyroPercent;
    
    data->pitch = ( ( data->pitch + data->gyroData[ 0 ] * dt ) * gyroPercent ) + ( data->pitchAccDeg * accelPercent ); // Angle around the X-axis
    data->roll  = ( ( data->roll  - data->gyroData[ 1 ] * dt ) * gyroPercent ) + ( data->rollAccDeg  * accelPercent );
    data->yaw   = ( data->gyroData[ 2 ] ) * dt; // Angle around the X-axis

    data->pitchWhole = data->pitch;
    data->rollWhole = data->roll;
    data->yawWhole = data->yaw;
    
    
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
	BSP_INIT_GYRO();
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
    float sampleRate;

    Timer_Init( &mAccelTimer, ACCEL_TIMER_MSEC );
    Timer_Init( &mLcdTimer, 100 );
    Timer_Init( &mMotorTimer, 1 );
    Timer_Init( &mLedTimer, 100 );

    Orientation_Init( &mOrientationData );

    sampleRate = ( 1000.0f / ((float)ACCEL_TIMER_MSEC) );
    Pid_Init( &mPidAngle, 100, 0, 0, sampleRate, 1000, 1000 );

//    while( 1 )
//    {
//      if( Timer_Run( &mLedTimer ) )
//      {
//        static int counterLed = 0;
//        
//        counterLed++;
//        
//        if( counterLed % 2 == 0 )
//        {
//            BSP_LED_Toggle( LED_GREEN );
//        }
//        if( counterLed % 4 == 0 )
//        {
//            BSP_LED_Toggle( LED_RED );
//        }
//        if( counterLed % 8 == 0 )
//        {
//            BSP_LED_Toggle( LED_BLUE );
//        }
//        
//        
//      }
//    }
    
    while( 1 )
    {
        static int counter = 0;
        static float gyroCalibration[ 3 ] = { 0,0,0 };
        static float gyroBuf[ 3 ];

    //calibrating
        if( Timer_Run( &mAccelTimer ) )
        {

            BSP_GYRO_GetXYZ( gyroBuf );

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
            float lastRollAcc = mOrientationData.rollAccDeg;
            
            Orientation_Calc1( &mOrientationData );

            lastOut = currentOut;
            currentOut = Pid_Run( &mPidAngle, 0 - mOrientationData.roll );
            
            if( currentOut > 20 )
            {
              currentOut = currentOut < 100 ? 100 : currentOut;
            } 
            else if( currentOut < -20 )
            {
              currentOut = currentOut > -100 ? -100 : currentOut;
            }
            
            if( mOrientationData.roll > 50 || 
                mOrientationData.roll < -50 )
            {
                MotorDriver_SetSpeed( &mMotorDriverB, 0 );
            }
            else
            {
                MotorDriver_SetSpeed( &mMotorDriverB, currentOut );
            }
            
//            DrawGraph( INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, LCD_COLOR_BLACK );
//            //DrawGraph( (int16_t)(mOrientationData.roll * 10), (int16_t)(lastRoll * 10), -1800, 1800, LCD_COLOR_RED );
//            //DrawGraph( ( int16_t )( mOrientationData.rollAcc * 10 ), ( int16_t )( lastRollAcc * 10 ), -1800, 1800, LCD_COLOR_BLUE );
//            DrawGraph( ( int16_t )( currentOut / 2 ),(int16_t)( lastOut/2 ), mPidAngle.limitMin, mPidAngle.limitMax, LCD_COLOR_GREEN );
//            
//            currentPos++;
//            
//            DrawGraph( INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, LCD_COLOR_GREEN );
//            //sprintf( string, "%d", mZ );
//            //BSP_LCD_ClearStringLine( 3 );
//            //BSP_LCD_DisplayStringAtLine( 3, string );
        }
    }
    
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
/******************************* I2C Routines**********************************/
/**
* @brief  Configures I2C interface.
*/
static I2C_HandleTypeDef    I2cHandle;

/*############################# I2C1 #########################################*/
/* I2C clock speed configuration (in Hz) */
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED                            100000
#endif /* BSP_I2C_SPEED */

/* I2C peripheral configuration defines (control interface of the audio codec) */
#define DISCOVERY_I2Cx                            I2C2
#define DISCOVERY_I2Cx_CLK_ENABLE()               __I2C2_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOF_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C2
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT          GPIOF
#define DISCOVERY_I2Cx_SCL_PIN                    GPIO_PIN_1
#define DISCOVERY_I2Cx_SDA_PIN                    GPIO_PIN_0

#define DISCOVERY_I2Cx_FORCE_RESET()              __I2C2_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()            __I2C2_RELEASE_RESET()

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                    I2C2_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                    I2C2_ER_IRQn

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
on accurate values, they just guarantee that the application will not remain
stuck if the SPI communication is corrupted.
You may modify these timeout values depending on CPU frequency and application
conditions (interrupts routines ...). */
#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */
#define I2Cx_MAX_COMMUNICATION_FREQ             ((uint32_t) 100000)

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;

/* I2Cx bus function */
static void    I2Cx_Init(void);
static void    I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg);
static void    I2Cx_Error (void);
static void    I2Cx_MspInit(I2C_HandleTypeDef *hi2c);

/* Link function for COMPASS / ACCELERO peripheral */
void    COMPASSACCELERO_IO_Init(void);
void    COMPASSACCELERO_IO_ITConfig(void);
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr);

static void I2Cx_Init(void)
{
	if (HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
	{
		I2cHandle.Instance = DISCOVERY_I2Cx;
		I2cHandle.Init.OwnAddress1 = 0x00;
		I2cHandle.Init.ClockSpeed = I2Cx_MAX_COMMUNICATION_FREQ;
		I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
		I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		I2cHandle.Init.OwnAddress2 = 0x00;
		I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

		/* Init the I2C */
		I2Cx_MspInit(&I2cHandle);
		HAL_I2C_Init(&I2cHandle);
	}
}

/**
* @brief  Writes a value in a register of the device through BUS.
* @param  Addr: Device address on BUS Bus.
* @param  Reg: The target register address to write
* @param  Value: The target register value to be written
*/
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error();
	}
}

/**
* @brief  Reads a register of the device through BUS.
* @param  Addr: Device address on BUS Bus.
* @param  Reg: The target register address to write
* @retval Data read at register address
*/
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;

	status = HAL_I2C_Mem_Read(&I2cHandle, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error();
	}
	return value;
}

/**
* @brief  I2Cx error treatment function.
*/
static void I2Cx_Error(void)
{
	/* De-initialize the I2C comunication BUS */
	HAL_I2C_DeInit(&I2cHandle);

	/* Re- Initiaize the I2C comunication BUS */
	I2Cx_Init();
}

/**
* @brief  I2Cx MSP Init.
* @param  hi2c: I2C handle
*/
static void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the I2C peripheral */
	DISCOVERY_I2Cx_CLK_ENABLE();

	/* Enable SCK and SDA GPIO clocks */
	DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

	/* I2Cx SD1 & SCK pin configuration */
	GPIO_InitStructure.Pin = DISCOVERY_I2Cx_SDA_PIN | DISCOVERY_I2Cx_SCL_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = DISCOVERY_I2Cx_SCL_SDA_AF;

	HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Force the I2C peripheral clock reset */
	DISCOVERY_I2Cx_FORCE_RESET();

	/* Release the I2C peripheral clock reset */
	DISCOVERY_I2Cx_RELEASE_RESET();

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);
}
void COMPASSACCELERO_IO_Init(void)
{
	I2Cx_Init();
}

/**
* @brief  Configures COMPASS / ACCELERO click IT.
*/
void COMPASSACCELERO_IO_ITConfig(void)
{
	
}

void COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
}

uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
  return I2Cx_ReadData(DeviceAddr, RegisterAddr);
}


void GYRO_IO_Init(void)
{
	I2Cx_Init();
}

/**
* @brief  Configures COMPASS / ACCELERO click IT.
*/
void GYRO_IO_DeInit(void)
{

}

void GYRO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
}

uint8_t GYRO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
	return I2Cx_ReadData(DeviceAddr, RegisterAddr);
}


void BSP_INIT_GYRO()
{
	mpGyroDriver->Init((L3GD20H_CTRL1_PD_NORMAL | L3GD20H_CTRL1_XEN | L3GD20H_CTRL1_YEN | L3GD20H_CTRL1_ZEN | L3GD20H_CTRL1_BW_MED_LOW | L3GD20H_CTRL1_RATE_800_50) |
		((L3GD20H_CTRL4_FS_250) << 8));
}

void BSP_GYRO_GetXYZ(float * gyroBuf)
{
	mpGyroDriver->GetXYZ(gyroBuf);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
