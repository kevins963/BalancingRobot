#include "robot/core/i2c_dev.h"

void I2CDev_Init(I2CDev* p_this, const I2cFunctions* i2c_functions) {
  InitClass(I2CDev);
  p_this->v_functions = *i2c_functions;
}

void I2CDev_Write(I2CDev* p_this, const I2cRequest* request) {
  p_this->v_functions.WriteRegister(p_this, request);
}

void I2CDev_Read(I2CDev* p_this, const I2cRequest* request) {
  p_this->v_functions.ReadRegister(p_this, request);
}
