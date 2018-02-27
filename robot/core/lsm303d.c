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

void LSM303D_ReadRegister(LSM303D* p_this, I2cRequest* request,
                          uint32_t start_reg, uint32_t end_reg) {
  LSM303D_DefaultRequest(p_this, request, start_reg, end_reg);
  request->buf_reg = &p_this->read_buf[start_reg];
  I2CDev_Read(&p_this->i2c_dev, request);
}

void LSM303D_WriteRegister(LSM303D* p_this, I2cRequest* request,
                           uint32_t start_reg, uint32_t end_reg) {
  LSM303D_DefaultRequest(p_this, request, start_reg, end_reg);
  request->buf_reg = &p_this->write_buf[start_reg];
  I2CDev_Write(&p_this->i2c_dev, request);
}

void LSM303D_Init(LSM303D* p_this, const I2cFunctions* i2c_functions) {
  InitClass(LSM303D);
  I2CDev_Init(&p_this->i2c_dev, i2c_functions);
  p_this->device_address = LSM303D_DEVICE_ADDR;
}

void LSM303D_Start(LSM303D* p_this) {}
void LSM303D_ReadAll(LSM303D* p_this) {}
void LSM303D_ReadWhoAmI(LSM303D* p_this) {
  LSM303D_ReadRegister(p_this, &request, who_am_i, who_am_i);
}

void LSM303D_OnComplete(LSM303D* p_this) { return; }
