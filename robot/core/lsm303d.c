#include "robot/core/lsm303d.h"

#include <assert.h>

void LSM303D_DefaultRequest(LSM303D* p_this, I2cRequest* out_request,
                            uint32_t start_reg, uint32_t end_reg) {
  assert(end_reg - start_reg < LSM303D_RA_MAX_REGISTERS);
  assert(end_reg >= start_reg);
  out_request->start_reg_adr = start_reg | 0x80;
  out_request->size = end_reg - start_reg + 1;
  out_request->device_address = p_this->device_address;
}

void LSM303D_ReadRegister(LSM303D* p_this, uint32_t start_reg,
                          uint32_t end_reg) {
  I2cRequest request;
  LSM303D_DefaultRequest(p_this, &request, start_reg, end_reg);
  request.buf_reg = &p_this->read_buf[start_reg];
  I2CDev_Read(&p_this->i2c_dev, &request);
}

void LSM303D_WriteRegister(LSM303D* p_this, uint32_t start_reg,
                           uint32_t end_reg) {
  I2cRequest request;
  LSM303D_DefaultRequest(p_this, &request, start_reg, end_reg);
  request.buf_reg = &p_this->write_buf[start_reg];
  I2CDev_Write(&p_this->i2c_dev, &request);
}

void LSM303D_Init(LSM303D* p_this, const I2cFunctions* i2c_functions) {
  InitClass(LSM303D);
  I2CDev_Init(&p_this->i2c_dev, i2c_functions);
  p_this->device_address = LSM303D_DEVICE_ADDR;
}

void LSM303D_Start(LSM303D* p_this) {
  p_this->write_buf[LSM303D_RA_CTRL0] = 0;
  p_this->write_buf[LSM303D_RA_CTRL1] =
      LSM303D_VAL_CTRL1_ODR_800 | LSM303D_VAL_CTRL1_X_EN |
      LSM303D_VAL_CTRL1_Y_EN | LSM303D_VAL_CTRL1_Z_EN;
  p_this->write_buf[LSM303D_RA_CTRL2] = LSM303D_VAL_CTRL2_SCALE_2G;
  p_this->write_buf[LSM303D_RA_CTRL3] = 0;
  p_this->write_buf[LSM303D_RA_CTRL4] = 0;
  p_this->write_buf[LSM303D_RA_CTRL5] = 0;
  p_this->write_buf[LSM303D_RA_CTRL6] = 0;
  p_this->write_buf[LSM303D_RA_CTRL7] = LSM303D_VAL_CTRL7_M_MODE_PD;
  LSM303D_WriteRegister(p_this, LSM303D_RA_CTRL0, LSM303D_RA_CTRL7);
}
void LSM303D_ReadAll(LSM303D* p_this) {
  LSM303D_ReadRegister(p_this, LSM303D_RA_INT_CTRL_M, LSM303D_RA_ACT_DUR);
}

void LSM303D_ReadWhoAmI(LSM303D* p_this) {
  LSM303D_ReadRegister(p_this, LSM303D_RA_WHO_AM_I_ADDR,
                       LSM303D_RA_WHO_AM_I_ADDR);
}

void LSM303D_ReadAccel(LSM303D* p_this) {
  LSM303D_ReadRegister(p_this, LSM303D_RA_STATUS_A, LSM303D_RA_OUT_Z_H_A);
}

void LSM303D_OnComplete(LSM303D* p_this) {
  p_this->accel_data.x =
      (float)(int16_t)(p_this->read_buf[LSM303D_RA_OUT_X_H_A] << 8 |
                       p_this->read_buf[LSM303D_RA_OUT_X_L_A]) *
      LSM303D_SENSITIVITY_2G;
  p_this->accel_data.y =
      (float)(int16_t)(p_this->read_buf[LSM303D_RA_OUT_Y_H_A] << 8 |
                       p_this->read_buf[LSM303D_RA_OUT_Y_L_A]) *
      LSM303D_SENSITIVITY_2G;
  p_this->accel_data.z =
      (float)(int16_t)(p_this->read_buf[LSM303D_RA_OUT_Z_H_A] << 8 |
                       p_this->read_buf[LSM303D_RA_OUT_Z_L_A]) *
      LSM303D_SENSITIVITY_2G;
}
