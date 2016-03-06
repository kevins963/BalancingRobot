/**
  ******************************************************************************
  * @file    LSM303D.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    24-June-2015
  * @brief   This file contains all the functions prototypes for the LSM303D.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM303D_H
#define __LSM303D_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/accelero.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
   
/** @addtogroup LSM303D
  * @{
  */
  
/** @defgroup LSM303D_Exported_Types
  * @{
  */

/**
  * @}
  */
 
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
/* Exported constant IO ------------------------------------------------------*/
#define ACC_I2C_ADDRESS_LO                0x3C
#define ACC_I2C_ADDRESS_HI                0x3A

#define I_AM_LSM303D                     ((uint8_t)0x49)


#define LSM303D_REG_TEMP_OUT_L 			0x05
#define LSM303D_REG_TEMP_OUT_H 			0x06
#define LSM303D_REG_STATUS_M 			0x07
#define LSM303D_REG_OUT_X_L_M 			0x08
#define LSM303D_REG_OUT_X_H_M 			0x09
#define LSM303D_REG_OUT_Y_L_M 			0x0A
#define LSM303D_REG_OUT_Y_H_M 			0x0B
#define LSM303D_REG_OUT_Z_L_M 			0x0C
#define LSM303D_REG_OUT_Z_H_M 			0x0D
#define LSM303D_REG_WHO_AM_I 			0x0F
#define LSM303D_REG_INT_CTRL_M 			0x12
#define LSM303D_REG_INT_SRC_M 			0x13
#define LSM303D_REG_INT_THS_L_M 		0x14
#define LSM303D_REG_INT_THS_H_M 		0x15
#define LSM303D_REG_OFFSET_X_L_M 		0x16
#define LSM303D_REG_OFFSET_X_H_M 		0x17
#define LSM303D_REG_OFFSET_Y_L_M 		0x18
#define LSM303D_REG_OFFSET_Y_H_M 		0x19
#define LSM303D_REG_OFFSET_Z_L_M 		0x1A
#define LSM303D_REG_OFFSET_Z_H_M 		0x1B
#define LSM303D_REG_REFERENCE_X 		0x1C
#define LSM303D_REG_REFERENCE_Y 		0x1D
#define LSM303D_REG_REFERENCE_Z 		0x1E
#define LSM303D_REG_CTRL0 				0x1F
#define LSM303D_REG_CTRL1 				0x20
#define LSM303D_REG_CTRL2 				0x21
#define LSM303D_REG_CTRL3 				0x22
#define LSM303D_REG_CTRL4 				0x23
#define LSM303D_REG_CTRL5 				0x24
#define LSM303D_REG_CTRL6 				0x25
#define LSM303D_REG_CTRL7 				0x26
#define LSM303D_REG_STATUS_A 			0x27
#define LSM303D_REG_OUT_X_L_A 			0x28
#define LSM303D_REG_OUT_X_H_A 			0x29
#define LSM303D_REG_OUT_Y_L_A 			0x2A
#define LSM303D_REG_OUT_Y_H_A 			0x2B
#define LSM303D_REG_OUT_Z_L_A 			0x2C
#define LSM303D_REG_OUT_Z_H_A 			0x2D
#define LSM303D_REG_FIFO_CTRL 			0x2E
#define LSM303D_REG_FIFO_SRC 			0x2F
#define LSM303D_REG_IG_CFG1 			0x30
#define LSM303D_REG_IG_SRC1 			0x31
#define LSM303D_REG_IG_THS1 			0x32
#define LSM303D_REG_IG_DUR1 			0x33
#define LSM303D_REG_IG_CFG2 			0x34
#define LSM303D_REG_IG_SRC2 			0x35
#define LSM303D_REG_IG_THS2 			0x36
#define LSM303D_REG_IG_DUR2 			0x37
#define LSM303D_REG_CLICK_CFG 			0x38
#define LSM303D_REG_CLICK_SRC 			0x39
#define LSM303D_REG_CLICK_THS 			0x3A
#define LSM303D_REG_TIME_LIMIT 			0x3B
#define LSM303D_REG_TIME_LATENCY 		0x3C
#define LSM303D_REG_TIME_WINDOW 		0x3D
#define LSM303D_REG_ACT_THS 			0x3E
#define LSM303D_REG_ACT_DUR 			0x3F
/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/


//CTRL0
#define LSM303D_CTRL0_BOOT_NORMAL ((uint8_t)0x00)
#define LSM303D_CTRL0_BOOT_REBOOT ((uint8_t)0x80)

#define LSM303D_CTRL0_FIFO_ENABLED  ((uint8_t)0x40)
#define LSM303D_CTRL0_FIFO_DISABLED ((uint8_t)0x00)

#define LSM303D_CTRL0_FIFO_THRESH_ENABLED  ((uint8_t)0x20)
#define LSM303D_CTRL0_FIFO_THRESH_DISABLED ((uint8_t)0x00)

#define LSM303D_CTRL0_HPF_CLICK_ENABLED  ((uint8_t)0x04)
#define LSM303D_CTRL0_HPF_CLICK_DISABLED ((uint8_t)0x00)

#define LSM303D_CTRL0_HPF1_INT_ENABLED  ((uint8_t)0x02)
#define LSM303D_CTRL0_HPF1_INT_DISABLED ((uint8_t)0x00)

#define LSM303D_CTRL0_HPF2_INT_ENABLED  ((uint8_t)0x01)
#define LSM303D_CTRL0_HPF2_INT_DISABLED ((uint8_t)0x00)

#define LSM303D_CTRL1_AODR_POWER_DOWN ((uint8_t)0x00)
#define LSM303D_CTRL1_AODR_3_125_HZ ((uint8_t)0x10)
#define LSM303D_CTRL1_AODR_6_25_HZ ((uint8_t)0x20)
#define LSM303D_CTRL1_AODR_12_5_HZ ((uint8_t)0x30)
#define LSM303D_CTRL1_AODR_25_HZ ((uint8_t)0x40)
#define LSM303D_CTRL1_AODR_50_HZ ((uint8_t)0x50)
#define LSM303D_CTRL1_AODR_100_HZ ((uint8_t)0x60)
#define LSM303D_CTRL1_AODR_200_HZ ((uint8_t)0x70)
#define LSM303D_CTRL1_AODR_400_HZ ((uint8_t)0x80)
#define LSM303D_CTRL1_AODR_800_HZ ((uint8_t)0x90)
#define LSM303D_CTRL1_AODR_1600_HZ ((uint8_t)0xA0)

#define LSM303D_CNTR1_BDU_DATA_CONTINIOUS ((uint8_t)0x00)
#define LSM303D_CNTR1_BDU_WAIT_TILL_READ ((uint8_t)0x08)

#define LSM303D_CNTR1_Z_ENABLED  ((uint8_t)0x04)
#define LSM303D_CNTR1_Z_DISABLED ((uint8_t)0x00)

#define LSM303D_CNTR1_Y_ENABLED  ((uint8_t)0x02)
#define LSM303D_CNTR1_Y_DISABLED ((uint8_t)0x00)

#define LSM303D_CNTR1_X_ENABLED  ((uint8_t)0x01)
#define LSM303D_CNTR1_X_DISABLED ((uint8_t)0x00)

#define LSM303D_CNTR2_ACCEL_FILTER_773_HZ ((uint8_t)0x00)
#define LSM303D_CNTR2_ACCEL_FILTER_194_HZ ((uint8_t)0x40)
#define LSM303D_CNTR2_ACCEL_FILTER_362_HZ ((uint8_t)0x80)
#define LSM303D_CNTR2_ACCEL_FILTER_50_HZ  ((uint8_t)0xC0)

#define LSM303D_CNTR2_ACCEL_SCALE_G_2  ((uint8_t)0x00)
#define LSM303D_CNTR2_ACCEL_SCALE_G_4  ((uint8_t)0x08)
#define LSM303D_CNTR2_ACCEL_SCALE_G_6  ((uint8_t)0x10)
#define LSM303D_CNTR2_ACCEL_SCALE_G_8  ((uint8_t)0x18)
#define LSM303D_CNTR2_ACCEL_SCALE_G_16 ((uint8_t)0x20)
#define LSM303D_CNTR2_ACCEL_SCALE_MASK ((uint8_t)0x38)

#define LSM303D_CNTR2_ACCEL_SELF_TEST_ENABLED  ((uint8_t)0x02)
#define LSM303D_CNTR2_ACCEL_SELF_TEST_DISABLED ((uint8_t)0x00)

#define LSM303D_CNTR2_SPI_4WIRE  ((uint8_t)0x01)
#define LSM303D_CNTR2_SPI_3WIRE  ((uint8_t)0x00)

#define LSM303D_CNTR5_TEMP_ENABLED              ((uint8_t)0x80)
#define LSM303D_CNTR5_TEMP_DISABLED             ((uint8_t)0x00)

#define LSM303D_CNTR5_INT_LATCH2_LATCHED        ((uint8_t)0x02)
#define LSM303D_CNTR5_INT_LATCH2_NOT_LATCHED    ((uint8_t)0x00)
#define LSM303D_CNTR5_INT_LATCH1_LATCHED        ((uint8_t)0x01)
#define LSM303D_CNTR5_INT_LATCH1_NOT_LATCHED    ((uint8_t)0x00)

#define LSM303D_CNTR5_MAG_RES_LOW  ((uint8_t)0x00)
#define LSM303D_CNTR5_MAG_RES_HIGH ((uint8_t)0x60)

#define LSM303D_CNTR5_MAG_ODR_3_125_HZ ((uint8_t)0x00)
#define LSM303D_CNTR5_MAG_ODR_6_25_HZ  ((uint8_t)0x08)
#define LSM303D_CNTR5_MAG_ODR_12_5_HZ  ((uint8_t)0x10)
#define LSM303D_CNTR5_MAG_ODR_25_HZ    ((uint8_t)0x18)
#define LSM303D_CNTR5_MAG_ODR_50_HZ    ((uint8_t)0x20)
#define LSM303D_CNTR5_MAG_ODR_100_HZ   ((uint8_t)0x28)

#define LSM303D_CNTR6_ACCEL_HPM_NORMAL      ((uint8_t)0x00)
#define LSM303D_CNTR6_ACCEL_HPM_REF         ((uint8_t)0x40)
//#define LSM303D_CNTR6_ACCEL_HPM_NORMAL      ((uint8_t)0x80)
#define LSM303D_CNTR6_ACCEL_HPM_AUTO_RESET  ((uint8_t)0xC0)

#define LSM303D_CNTR6_ACCEL_FILTER_ENABLED  ((uint8_t)0x20)
#define LSM303D_CNTR6_ACCEL_FILTER_DISABLED ((uint8_t)0x00)

#define LSM303D_CNTR6_TEMP_NORMAL    ((uint8_t)0x00)
#define LSM303D_CNTR6_TEMP_ALWAYS_ON ((uint8_t)0x10)

#define LSM303D_CNTR6_MAG_MLP_LOW_POWER ((uint8_t)0x04)
#define LSM303D_CNTR6_MAG_MLP_NORMAL    ((uint8_t)0x00)

#define LSM303D_CNTR6_MAG_MODE_CONTINIOUS   ((uint8_t)0x00)
#define LSM303D_CNTR6_MAG_MODE_SINGLE       ((uint8_t)0x01)
#define LSM303D_CNTR6_MAG_MODE_POWER_DOWN   ((uint8_t)0x02)
#define LSM303D_CNTR6_MAG_MODE_POWER_DOWN2  ((uint8_t)0x03)

#define LSM303D_ ((uint8_t)0x)
#define LSM303D_ ((uint8_t)0x)
#define LSM303D_ ((uint8_t)0x)
#define LSM303D_ ((uint8_t)0x)
#define LSM303D_ ((uint8_t)0x)
/** @defgroup Acc_Full_Scale_Selection
  * @{
  */
#define LSM303D_ACC_SENSITIVITY_2G     ((uint32_t)61)  /*!< accelerometer sensitivity with 2 g full scale [ug/LSB] */
#define LSM303D_ACC_SENSITIVITY_4G     ((uint32_t)122)  /*!< accelerometer sensitivity with 4 g full scale [ug/LSB] */
#define LSM303D_ACC_SENSITIVITY_6G     ((uint32_t)183)  /*!< accelerometer sensitivity with 8 g full scale [ug/LSB] */
#define LSM303D_ACC_SENSITIVITY_8G     ((uint32_t)244)  /*!< accelerometer sensitivity with 8 g full scale [ug/LSB] */
#define LSM303D_ACC_SENSITIVITY_16G    ((uint32_t)732) /*!< accelerometer sensitivity with 12 g full scale [ug/LSB] */
/**
  * @}
  */

/**
 * @defgroup Magnetometer_Sensitivity
 * @{
 */
#define LSM303D_M_SENSITIVITY_UG_LSB_2  ((uint32_t)80) //ugauss/lsb
#define LSM303D_M_SENSITIVITY_UG_LSB_4  ((uint32_t)160) //ugauss/lsb
#define LSM303D_M_SENSITIVITY_UG_LSB_8  ((uint32_t)320) //ugauss/lsb
#define LSM303D_M_SENSITIVITY_UG_LSB_12 ((uint32_t)479) //ugauss/lsb
 
/**
  * @}
  */
  
/** @defgroup LSM303D_Exported_Functions
  * @{
  */
/* ACC functions */
void    LSM303D_AccInit( uint16_t cntl );
void    LSM303D_AccDeInit(void);
uint8_t LSM303D_AccReadID(void);
void    LSM303D_AccRebootCmd(void);
void    LSM303D_AccFilterConfig(uint8_t FilterStruct);
void    LSM303D_AccFilterCmd(uint8_t HighPassFilterState);
void    LSM303D_AccReadXYZ(int16_t* pData);
void    LSM303D_AccFilterClickCmd(uint8_t HighPassFilterClickState);
void    LSM303D_AccIT1Enable(uint8_t LSM303D_IT);
void    LSM303D_AccIT1Disable(uint8_t LSM303D_IT);
void    LSM303D_AccIT2Enable(uint8_t LSM303D_IT);
void    LSM303D_AccIT2Disable(uint8_t LSM303D_IT);
void    LSM303D_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303D_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303D_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303D_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303D_AccClickITEnable(uint8_t ITClick);
void    LSM303D_AccClickITDisable(uint8_t ITClick);
void    LSM303D_AccZClickITConfig(void);

void    LSM303D_ConfigSlaveAddress( uint8_t isHighAddress );
/* COMPASS / ACCELERO IO functions */
void    COMPASSACCELERO_IO_Init(void);
void    COMPASSACCELERO_IO_ITConfig(void);
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr);

/* ACC driver structure */
extern ACCELERO_DrvTypeDef LSM303DDrv;


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

#ifdef __cplusplus
}
#endif

#endif /* __LSM303D_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
