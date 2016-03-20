
/******************************* I2C Routines *********************************/

/**
* @brief  I2Cx MSP Initialization
* @param  hi2c: I2C handle
* @retval None
*/
static void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
#ifdef EE_M24LR64
	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;

	I2C_HandleTypeDef* pI2cHandle;
	pI2cHandle = &I2cHandle;
#endif /* EE_M24LR64 */

	if (hi2c->Instance == DISCOVERY_I2Cx)
	{
		/* Configure the GPIOs ---------------------------------------------------*/
		/* Enable GPIO clock */
		DISCOVERY_I2Cx_SDA_GPIO_CLK_ENABLE();
		DISCOVERY_I2Cx_SCL_GPIO_CLK_ENABLE();

		/* Configure I2C Tx as alternate function  */
		GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SCL_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = DISCOVERY_I2Cx_SCL_SDA_AF;
		HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

		/* Configure I2C Rx as alternate function  */
		GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SDA_PIN;
		HAL_GPIO_Init(DISCOVERY_I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);


		/* Configure the Discovery I2Cx peripheral -------------------------------*/
		/* Enable I2C3 clock */
		DISCOVERY_I2Cx_CLOCK_ENABLE();

		/* Force the I2C Peripheral Clock Reset */
		DISCOVERY_I2Cx_FORCE_RESET();

		/* Release the I2C Peripheral Clock Reset */
		DISCOVERY_I2Cx_RELEASE_RESET();

		/* Enable and set Discovery I2Cx Interrupt to the highest priority */
		HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0x00, 0);
		HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

		/* Enable and set Discovery I2Cx Interrupt to the highest priority */
		HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0x00, 0);
		HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);

#ifdef EE_M24LR64
		/* I2C DMA TX and RX channels configuration */
		/* Enable the DMA clock */
		EEPROM_I2C_DMA_CLK_ENABLE();

		/* Configure the DMA stream for the EE I2C peripheral TX direction */
		/* Configure the DMA Stream */
		hdma_tx.Instance = EEPROM_I2C_DMA_STREAM_TX;
		/* Set the parameters to be configured */
		hdma_tx.Init.Channel = EEPROM_I2C_DMA_CHANNEL;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_tx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

		/* Associate the initilalized hdma_tx handle to the the pI2cHandle handle */
		__HAL_LINKDMA(pI2cHandle, hdmatx, hdma_tx);

		/* Configure the DMA Stream */
		HAL_DMA_Init(&hdma_tx);

		/* Configure and enable I2C DMA TX Channel interrupt */
		HAL_NVIC_SetPriority((IRQn_Type)(EEPROM_I2C_DMA_TX_IRQn), EEPROM_I2C_DMA_PREPRIO, 0);
		HAL_NVIC_EnableIRQ((IRQn_Type)(EEPROM_I2C_DMA_TX_IRQn));

		/* Configure the DMA stream for the EE I2C peripheral TX direction */
		/* Configure the DMA Stream */
		hdma_rx.Instance = EEPROM_I2C_DMA_STREAM_RX;
		/* Set the parameters to be configured */
		hdma_rx.Init.Channel = EEPROM_I2C_DMA_CHANNEL;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

		/* Associate the initilalized hdma_rx handle to the the pI2cHandle handle*/
		__HAL_LINKDMA(pI2cHandle, hdmarx, hdma_rx);

		/* Configure the DMA Stream */
		HAL_DMA_Init(&hdma_rx);

		/* Configure and enable I2C DMA RX Channel interrupt */
		HAL_NVIC_SetPriority((IRQn_Type)(EEPROM_I2C_DMA_RX_IRQn), EEPROM_I2C_DMA_PREPRIO, 0);
		HAL_NVIC_EnableIRQ((IRQn_Type)(EEPROM_I2C_DMA_RX_IRQn));
#endif /* EE_M24LR64 */
	}
}

/**
* @brief  I2Cx Bus initialization.
* @param  None
* @retval None
*/
static void I2Cx_Init(void)
{
	if (HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
	{
		I2cHandle.Instance = DISCOVERY_I2Cx;
		I2cHandle.Init.ClockSpeed = BSP_I2C_SPEED;
		I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
		I2cHandle.Init.OwnAddress1 = 0;
		I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		I2cHandle.Init.OwnAddress2 = 0;
		I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

		/* Init the I2C */
		I2Cx_MspInit(&I2cHandle);
		HAL_I2C_Init(&I2cHandle);
	}
}

/**
* @brief  Configures Interruption pin for I2C communication.
* @param  None
* @retval None
*/
static void I2Cx_ITConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO EXTI Clock */
	STMPE811_INT_CLK_ENABLE();

	GPIO_InitStruct.Pin = STMPE811_INT_PIN;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(STMPE811_INT_GPIO_PORT, &GPIO_InitStruct);

	/* Enable and set GPIO EXTI Interrupt to the highest priority */
	HAL_NVIC_SetPriority((IRQn_Type)(STMPE811_INT_EXTI), 0x00, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(STMPE811_INT_EXTI));
}

/**
* @brief  Writes a value in a register of the device through BUS.
* @param  Addr: Device address on BUS Bus.
* @param  Reg: The target register address to write
* @param  Value: The target register value to be written
* @retval None
*/
static void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		I2Cx_Error();
	}
}

/**
* @brief  Writes a value in a register of the device through BUS.
* @param  Addr: Device address on BUS Bus.
* @param  Reg: The target register address to write
* @param  pBuffer: The target register value to be written
* @param  Length: buffer size to be written
* @retval None
*/
static void I2Cx_WriteBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2cxTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		I2Cx_Error();
	}
}

/**
* @brief  Reads a register of the device through BUS.
* @param  Addr: Device address on BUS Bus.
* @param  Reg: The target register address to write
* @retval Data read at register address
*/
static uint8_t I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;

	status = HAL_I2C_Mem_Read(&I2cHandle, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Re-Initialize the BUS */
		I2Cx_Error();

	}
	return value;
}

/**
* @brief  Reads multiple data on the BUS.
* @param  Addr: I2C Address
* @param  Reg: Reg Address
* @param  pBuffer: pointer to read data buffer
* @param  Length: length of the data
* @retval 0 if no problems to read multiple data
*/
static uint8_t I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2cxTimeout);

	/* Check the communication status */
	if (status == HAL_OK)
	{
		return 0;
	}
	else
	{
		/* Re-Initialize the BUS */
		I2Cx_Error();

		return 1;
	}
}