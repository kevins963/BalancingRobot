#include "mems.h"
#include "Integers.h"
//Interface

//HAL
static ACCELERO_DrvTypeDef *mpAccelDriverInstance;

uint8_t Mems_Init( void )
{
    uint8_t ret = ACCELERO_ERROR;
    uint16_t ctrl = 0x0000;

    ACCELERO_InitTypeDef LSM303DLHC_InitStructure;
    ACCELERO_FilterConfigTypeDef LSM303DLHC_FilterStructure = { 0,0,0,0 };

    LSM303D_ConfigSlaveAddress( 1 );

    if( LSM303DDrv.ReadID() == I_AM_LSM303D )
    {
        /* Initialize the Accelerometer driver structure */
        mpAccelDriverInstance = &LSM303DDrv;

        /* MEMS configuration ----------------------------------------------------*/
        /* Fill the Accelerometer structure */
        LSM303DLHC_InitStructure.Power_Mode = 0;
        LSM303DLHC_InitStructure.AccOutput_DataRate = LSM303D_CTRL1_AODR_100_HZ;
        LSM303DLHC_InitStructure.Axes_Enable = LSM303D_CNTR1_X_ENABLED | LSM303D_CNTR1_Y_ENABLED | LSM303D_CNTR1_Z_ENABLED;
        LSM303DLHC_InitStructure.AccFull_Scale = LSM303D_CNTR2_ACCEL_SCALE_G_2 << 8;
        LSM303DLHC_InitStructure.BlockData_Update = LSM303D_CNTR1_BDU_DATA_CONTINIOUS;
        LSM303DLHC_InitStructure.Endianness = 0;
        LSM303DLHC_InitStructure.High_Resolution = 0;

        /* Configure MEMS: data rate, power mode, full scale and axes */
        ctrl |= ( LSM303DLHC_InitStructure.Power_Mode | LSM303DLHC_InitStructure.AccOutput_DataRate |
            LSM303DLHC_InitStructure.Axes_Enable | LSM303DLHC_InitStructure.BlockData_Update );

        ctrl |= ( ( LSM303DLHC_InitStructure.AccFull_Scale ) << 8 );

        /* Configure the Accelerometer main parameters */
        mpAccelDriverInstance->Init( ctrl );

        ///* Fill the Accelerometer LPF structure */
        //LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
        //LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
        //LSM303DLHC_FilterStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
        //LSM303DLHC_FilterStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

        ///* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
        //ctrl = ( uint8_t )( LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection | \
        //    LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency | \
        //    LSM303DLHC_FilterStructure.HighPassFilter_AOI1 | \
        //    LSM303DLHC_FilterStructure.HighPassFilter_AOI2 );

        ///* Configure the Accelerometer LPF main parameters */
        //mpMemsDriverInstance->FilterConfig( ctrl );

        ret = ACCELERO_OK;
    }
    else
    {
        ret = ACCELERO_ERROR;
    }

    return ret;

}


/**
* @brief  Reboot memory content of Accelerometer.
* @param  None
* @retval None
*/
void Mems_Reset( void )
{
    if( mpAccelDriverInstance->Reset != NULL )
    {
        mpAccelDriverInstance->Reset();
    }
}

/**
* @brief  Configure Accelerometer click IT.
* @param  None
* @retval None
*/
void Mems_ITConfig( void )
{
    if( mpAccelDriverInstance->ConfigIT != NULL )
    {
        mpAccelDriverInstance->ConfigIT();
    }
}
 
/**
* @brief  Get XYZ axes acceleration.
* @param  pDataXYZ: Pointer to 3 angular acceleration axes.
*                   pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
* @retval None
*/
void Mems_GetXYZ( int16_t *pDataXYZ )
{
    if( mpAccelDriverInstance->GetXYZ != NULL )
    {
        mpAccelDriverInstance->GetXYZ( pDataXYZ );
    }
}

