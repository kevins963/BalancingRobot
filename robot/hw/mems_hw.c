#include "robot/hw/mems_hw.h"

#include "robot/hw/stm32f4xx_it.h"

#define MEMS_SCL_GPIO GPIOF
#define MEMS_SCL_GPIO_PIN GPIO_PIN_1
#define MEMS_SCL_GPIO_AF GPIO_AF4_I2C2
#define MEMS_SCL_GPIO_CLK() __HAL_RCC_GPIOF_CLK_ENABLE()

#define MEMS_SDA_GPIO GPIOF
#define MEMS_SDA_GPIO_PIN GPIO_PIN_0
#define MEMS_SDA_GPIO_AF GPIO_AF4_I2C2
#define MEMS_SDA_GPIO_CLK() __HAL_RCC_GPIOF_CLK_ENABLE()

#define MEMS_I2C I2C2
#define MEMS_I2C_CLK() __HAL_RCC_I2C2_CLK_ENABLE()

I2C_HandleTypeDef i2c_;
MemsHw* mems_hw_instance_;

void MemsHw_DeviceCallbacks(uint8_t address) {
  if (address == mems_hw_instance_->gyro_driver.device_address) {
    L3GD20H_OnComplete(&mems_hw_instance_->gyro_driver);
  } else if (address == mems_hw_instance_->accel_driver.device_address) {
    LSM303D_OnComplete(&mems_hw_instance_->accel_driver);
  }
}
void MemsHw_ReadRegister(I2CDev* p_this, const I2cRequest* request) {
  HAL_I2C_Mem_Read_IT(&i2c_, request->device_address, request->start_reg_adr,
                      I2C_MEMADD_SIZE_8BIT, request->buf_reg, request->size);
}

void MemsHw_WriteRegister(I2CDev* p_this, const I2cRequest* request) {
  HAL_I2C_Mem_Write_IT(&i2c_, request->device_address, request->start_reg_adr,
                       I2C_MEMADD_SIZE_8BIT, request->buf_reg, request->size);
}

void MemsHw_Config(MemsHw* p_this) {
  GPIO_InitTypeDef gpio;

  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF4_I2C2;

  MEMS_SCL_GPIO_CLK();
  gpio.Pin = MEMS_SCL_GPIO_PIN;
  HAL_GPIO_Init(MEMS_SCL_GPIO, &gpio);

  MEMS_SDA_GPIO_CLK();
  gpio.Pin = MEMS_SDA_GPIO_PIN;
  HAL_GPIO_Init(MEMS_SDA_GPIO, &gpio);

  I2C_HandleTypeDef i2c;

  i2c.Instance = MEMS_I2C;
  i2c.Init.ClockSpeed = 400000;
  i2c.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  i2c.Init.OwnAddress1 = 0;
  i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c.Init.OwnAddress2 = 0;
  i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  MEMS_I2C_CLK();

  HAL_I2C_Init(&i2c);

  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 1, 0);
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

  i2c_ = i2c;
}

void MemsHw_Init(MemsHw* p_this) {
  InitClass(MemsHw);
  I2cFunctions i2c_functions;
  i2c_functions.ReadRegister = MemsHw_ReadRegister;
  i2c_functions.WriteRegister = MemsHw_WriteRegister;
  L3GD20H_Init(&p_this->gyro_driver, &i2c_functions);
  LSM303D_Init(&p_this->accel_driver, &i2c_functions);

  MemsHw_Config(p_this);

  mems_hw_instance_ = p_this;
}

void I2C2_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&i2c_); }
void I2C2_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&i2c_); }

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  MemsHw_DeviceCallbacks(hi2c->Devaddress);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  MemsHw_DeviceCallbacks(hi2c->Devaddress);
}
