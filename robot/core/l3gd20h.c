/**
******************************************************************************
* @file    L3GD20H.c
* @author  MCD Application Team
* @version V2.0.0
* @date    26-June-2015
* @brief   This file provides a set of functions needed to manage the L3GD20H,
*          ST MEMS motion sensor, 3-axis digital output gyroscope.
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
#include "L3GD20H.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup Components
* @{
*/

/** @addtogroup L3GD20H
* @{
*/

/** @defgroup L3GD20H_Private_TypesDefinitions
* @{
*/

/**
* @}
*/

/** @defgroup L3GD20H_Private_Defines
* @{
*/

/**
* @}
*/

/** @defgroup L3GD20H_Private_Macros
* @{
*/

/**
* @}
*/

/** @defgroup L3GD20H_Private_Variables
* @{
*/
GYRO_DrvTypeDef L3GD20HDrv =
{
    L3GD20H_Init,
    L3GD20H_DeInit,
    L3GD20H_ReadID,
    L3GD20H_RebootCmd,
    L3GD20H_LowPower,
    L3GD20H_INT1InterruptConfig,
    L3GD20H_EnableIT,
    L3GD20H_DisableIT,
    0,
    0,
    L3GD20H_FilterConfig,
    L3GD20H_FilterCmd,
    L3GD20H_ReadXYZAngRate
};

/**
* @}
*/

/** @defgroup L3GD20H_Private_FunctionPrototypes
* @{
*/

/**
* @}
*/

/** @defgroup L3GD20H_Private_Functions
* @{
*/

/**
* @brief  Set L3GD20H Initialization.
* @param  L3GD20H_InitStruct: pointer to a L3GD20H_InitTypeDef structure
*         that contains the configuration setting for the L3GD20H.
* @retval None
*/
void L3GD20H_Init( uint16_t InitStruct )
{
    uint8_t ctrl = 0x00;

    /* Configure the low level interface */
    GYRO_IO_Init();

    /* Write value to MEMS CTRL_REG1 register */
    ctrl = ( uint8_t )InitStruct;
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS , L3GD20H_RA_CTRL1, ctrl );

    /* Write value to MEMS CTRL_REG4 register */
    ctrl = ( uint8_t )( InitStruct >> 8 );
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL4, ctrl );
}



/**
* @brief L3GD20H De-initialization
* @param  None
* @retval None
*/
void L3GD20H_DeInit( void )
{
}

/**
* @brief  Read ID address of L3GD20H
* @param  None
* @retval ID name
*/
uint8_t L3GD20H_ReadID( void )
{
    uint8_t tmp;

    /* Configure the low level interface */
    GYRO_IO_Init();

    /* Read WHO I AM register */
    tmp = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_WHO_AM_I );

    /* Return the ID */
    return ( uint8_t )tmp;
}

/**
* @brief  Reboot memory content of L3GD20H
* @param  None
* @retval None
*/
void L3GD20H_RebootCmd( void )
{
    uint8_t tmpreg;

    /* Read CTRL_REG5 register */
    tmpreg = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL5 );

    /* Enable or Disable the reboot memory */
    tmpreg |= L3GD20H_CTRL5_BOOT_REBOOT;

    /* Write value to MEMS CTRL_REG5 register */
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL5, tmpreg );
}

/**
* @brief Set L3GD20H in low-power mode
* @param
* @retval  None
*/
void L3GD20H_LowPower( uint16_t InitStruct )
{
    uint8_t ctrl = 0x00;

    /* Write value to MEMS CTRL_REG1 register */
    ctrl = ( uint8_t )InitStruct;
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL1, ctrl );
}

/**
* @brief  Set L3GD20H Interrupt INT1 configuration
* @param  Int1Config: the configuration setting for the L3GD20H Interrupt.
* @retval None
*/
void L3GD20H_INT1InterruptConfig( uint16_t Int1Config )
{
    uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;

    ///* Read INT1_CFG register */
    //GYRO_IO_Read( &ctrl_cfr, L3GD20H_RA_, 1 );

    ///* Read CTRL_REG3 register */
    //GYRO_IO_Read( &ctrl3, L3GD20H_CTRL_REG3_ADDR, 1 );

    //ctrl_cfr &= 0x80;
    //ctrl_cfr |= ( ( uint8_t )Int1Config >> 8 );

    //ctrl3 &= 0xDF;
    //ctrl3 |= ( ( uint8_t )Int1Config );

    ///* Write value to MEMS INT1_CFG register */
    //GYRO_IO_Write( &ctrl_cfr, L3GD20H_INT1_CFG_ADDR, 1 );

    ///* Write value to MEMS CTRL_REG3 register */
    //GYRO_IO_Write( &ctrl3, L3GD20H_CTRL_REG3_ADDR, 1 );
}

/**
* @brief  Enable INT1 or INT2 interrupt
* @param  IntSel: choice of INT1 or INT2
*      This parameter can be:
*        @arg L3GD20H_INT1
*        @arg L3GD20H_INT2
* @retval None
*/
void L3GD20H_EnableIT( uint8_t IntSel )
{
    //uint8_t tmpreg;

    ///* Read CTRL_REG3 register */
    //GYRO_IO_Read( &tmpreg, L3GD20H_CTRL_REG3_ADDR, 1 );

    //if( IntSel == L3GD20H_INT1 )
    //{
    //    tmpreg &= 0x7F;
    //    tmpreg |= L3GD20H_INT1INTERRUPT_ENABLE;
    //}
    //else if( IntSel == L3GD20H_INT2 )
    //{
    //    tmpreg &= 0xF7;
    //    tmpreg |= L3GD20H_INT2INTERRUPT_ENABLE;
    //}

    ///* Write value to MEMS CTRL_REG3 register */
    //GYRO_IO_Write( &tmpreg, L3GD20H_CTRL_REG3_ADDR, 1 );
}

/**
* @brief  Disable  INT1 or INT2 interrupt
* @param  IntSel: choice of INT1 or INT2
*      This parameter can be:
*        @arg L3GD20H_INT1
*        @arg L3GD20H_INT2
* @retval None
*/
void L3GD20H_DisableIT( uint8_t IntSel )
{
    uint8_t tmpreg;

    ///* Read CTRL_REG3 register */
    //GYRO_IO_Read( &tmpreg, L3GD20H_CTRL_REG3_ADDR, 1 );

    //if( IntSel == L3GD20H_INT1 )
    //{
    //    tmpreg &= 0x7F;
    //    tmpreg |= L3GD20H_INT1INTERRUPT_DISABLE;
    //}
    //else if( IntSel == L3GD20H_INT2 )
    //{
    //    tmpreg &= 0xF7;
    //    tmpreg |= L3GD20H_INT2INTERRUPT_DISABLE;
    //}

    ///* Write value to MEMS CTRL_REG3 register */
    //GYRO_IO_Write( &tmpreg, L3GD20H_CTRL_REG3_ADDR, 1 );
}

/**
* @brief  Set High Pass Filter Modality
* @param  FilterStruct: contains the configuration setting for the L3GD20H.
* @retval None
*/
void L3GD20H_FilterConfig( uint8_t FilterStruct )
{
    uint8_t tmpreg;

    /* Read CTRL_REG2 register */
    
    tmpreg = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL2 );

    tmpreg &= ~(L3GD20H_CTRL2_HPM_MASK | L3GD20H_CTRL2_HPCF_MASK);

    /* Configure MEMS: mode and cutoff frequency */
    tmpreg |= FilterStruct;

    /* Write value to MEMS CTRL_REG2 register */
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL2, tmpreg );
}

/**
* @brief  Enable or Disable High Pass Filter
* @param  HighPassFilterState: new state of the High Pass Filter feature.
*      This parameter can be:
*         @arg: L3GD20H_HIGHPASSFILTER_DISABLE
*         @arg: L3GD20H_HIGHPASSFILTER_ENABLE
* @retval None
*/
void L3GD20H_FilterCmd( uint8_t HighPassFilterState )
{
    uint8_t tmpreg;

    /* Read CTRL_REG5 register */
    tmpreg = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL5 );

    tmpreg &= ~( L3GD20H_CTRL5_HPF_EN );

    tmpreg |= HighPassFilterState;

    /* Write value to MEMS CTRL_REG5 register */
    GYRO_IO_Write( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL5, tmpreg );
}

/**
* @brief  Get status for L3GD20H data
* @param  None
* @retval Data status in a L3GD20H Data
*/
uint8_t L3GD20H_GetDataStatus( void )
{
    uint8_t tmpreg;

    /* Read STATUS_REG register */
    tmpreg = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_STATUS );

    return tmpreg;
}

/**
* @brief  Calculate the L3GD20H angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void L3GD20H_ReadXYZAngRate( float *pfData )
{
    uint8_t tmpbuffer[ 6 ] = { 0 };
    int16_t RawData[ 3 ] = { 0 };
    uint8_t tmpreg = 0;
    float sensitivity = 0;
    int i = 0;

    tmpreg = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_CTRL4 );

    RawData[ 0 ] = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_X_H ) << 8;
    RawData[ 0 ] |= GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_X_L );
    RawData[ 1 ] = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_Y_H ) << 8;
    RawData[ 1 ] |= GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_Y_L );
    RawData[ 2 ] = GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_Z_H ) << 8;
    RawData[ 2 ] |= GYRO_IO_Read( L3GD20H_DEFAULT_ADDRESS, L3GD20H_RA_OUT_Z_L );

    /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
    //if( !( tmpreg & L3GD20H_CTRL4_BIG_ENDIAN ) )
    //{
    //    for( i = 0; i<3; i++ )
    //    {
    //        RawData[ i ] = ( int16_t )( ( ( uint16_t )tmpbuffer[ 2 * i + 1 ] << 8 ) + tmpbuffer[ 2 * i ] );
    //    }
    //}
    //else
    //{
    //    for( i = 0; i<3; i++ )
    //    {
    //        RawData[ i ] = ( int16_t )( ( ( uint16_t )tmpbuffer[ 2 * i ] << 8 ) + tmpbuffer[ 2 * i + 1 ] );
    //    }
    //}

    /* Switch the sensitivity value set in the CRTL4 */
    switch( tmpreg & L3GD20H_CTRL4_FS_MASK )
    {
    case L3GD20H_CTRL4_FS_250:
        sensitivity = L3GD20H_SENSITIVITY_250DPS;
        break;

    case L3GD20H_CTRL4_FS_500:
        sensitivity = L3GD20H_SENSITIVITY_500DPS;
        break;

    case L3GD20H_CTRL4_FS_2000:
        sensitivity = L3GD20H_SENSITIVITY_2000DPS;
        break;
    }
    /* Divide by sensitivity */
    for( i = 0; i<3; i++ )
    {
        pfData[ i ] = ( float )( ( RawData[ i ] ) * sensitivity );
    }
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
