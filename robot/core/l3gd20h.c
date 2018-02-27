#include "robot/core/l3gd20h.h"

#include <assert.h>

void L3GD20H_ReadRegister(L3GD20H* p_this, I2cRequest* request,
                          uint32_t start_reg, uint32_t end_reg) {
  assert(end_reg - start_reg < L3GD20H_REGISTERS);
  assert(end_reg >= start_reg);
  request->start_reg_adr = start_reg | 0x80;
  request->size = end_reg - start_reg + 1;
  request->device_address = p_this->device_address;
  request->buf_reg = &p_this->registers[start_reg];
  I2CDev_Read(&p_this->i2c_dev, request);
}

void L3GD20H_WriteRegister(L3GD20H* p_this, I2cRequest* request,
                           uint32_t start_reg, uint32_t end_reg) {
  assert(end_reg - start_reg < L3GD20H_REGISTERS);
  assert(end_reg >= start_reg);
  request->start_reg_adr = start_reg | 0x80;
  request->size = end_reg - start_reg + 1;
  request->device_address = p_this->device_address;
  request->buf_reg = &p_this->write_registers[start_reg];
  I2CDev_Write(&p_this->i2c_dev, request);
}

void L3GD20H_Init(L3GD20H* p_this, const I2cFunctions* i2c_functions) {
  InitClass(L3GD20H);
  I2CDev_Init(&p_this->i2c_dev, i2c_functions);
  p_this->device_address = L3GD20H_ADDRESS;
}

void L3GD20H_Start(L3GD20H* p_this) {
  I2cRequest request;
  p_this->write_registers[L3GD20H_RA_CTRL1] =
      L3GD20H_CTRL1_BW_HIGH | L3GD20H_CTRL1_RATE_800_50 |
      L3GD20H_CTRL1_PD_NORMAL | L3GD20H_CTRL1_XEN | L3GD20H_CTRL1_YEN |
      L3GD20H_CTRL1_ZEN;
  p_this->write_registers[L3GD20H_RA_CTRL2] = L3GD20H_CTRL2_HPCF7;
  p_this->write_registers[L3GD20H_RA_CTRL3] = 0;
  p_this->write_registers[L3GD20H_RA_CTRL4] = L3GD20H_CTRL4_FS_250;
  p_this->write_registers[L3GD20H_RA_CTRL5] = L3GD20H_CTRL5_OUT_HIGH_PASS;
  L3GD20H_WriteRegister(p_this, &request, L3GD20H_RA_CTRL1, L3GD20H_RA_CTRL5);
}

void L3GD20H_ReadAll(L3GD20H* p_this) {
  I2cRequest request;
  L3GD20H_ReadRegister(p_this, &request, L3GD20H_RA_CTRL1, L3GD20H_RA_LOW_ODR);
}

void L3GD20H_ReadWhoAmI(L3GD20H* p_this) {
  I2cRequest request;
  L3GD20H_ReadRegister(p_this, &request, L3GD20H_RA_WHO_AM_I,
                       L3GD20H_RA_WHO_AM_I);
}

void L3GD20H_ReadGyroData(L3GD20H* p_this) {
  I2cRequest request;
  L3GD20H_ReadRegister(p_this, &request, L3GD20H_RA_STATUS, L3GD20H_RA_OUT_Z_H);
}

void L3GD20H_OnComplete(L3GD20H* p_this) {
  p_this->who_am_i = p_this->registers[L3GD20H_RA_WHO_AM_I];
  p_this->gyro_data.x =
      (float)(int16_t)(p_this->registers[L3GD20H_RA_OUT_X_H] << 8 |
                       p_this->registers[L3GD20H_RA_OUT_X_L]) *
      L3GD20H_SENSITIVITY_250DPS;
  p_this->gyro_data.y =
      (float)(int16_t)(p_this->registers[L3GD20H_RA_OUT_Y_H] << 8 |
                       p_this->registers[L3GD20H_RA_OUT_Y_L]) *
      L3GD20H_SENSITIVITY_250DPS;
  p_this->gyro_data.z =
      (float)(int16_t)(p_this->registers[L3GD20H_RA_OUT_Z_H] << 8 |
                       p_this->registers[L3GD20H_RA_OUT_Z_L]) *
      L3GD20H_SENSITIVITY_250DPS;
}
