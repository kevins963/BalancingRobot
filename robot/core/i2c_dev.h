#ifndef ROBOT_CORE_I2C_DEV_H_
#define ROBOT_CORE_I2C_DEV_H_

#include "robot/core/integer.h"
#include "robot/core/util.h"

FORWARD_DECLARE_STRUCT(I2CDev)

BEGIN_STRUCT(I2cRequest)
uint8_t device_address;
uint8_t start_reg_adr;
int size;
uint8_t* buf_reg;
END_STRUCT(I2cRequest)

BEGIN_STRUCT(I2cFunctions)
void (*ReadRegister)(I2CDev* p_this, const I2cRequest* request);
void (*WriteRegister)(I2CDev* p_this, const I2cRequest* request);
END_STRUCT(I2cFunctions)
DEFINE_EMPTY_STRUCT(I2CDev)

struct I2CDev {
  I2cFunctions v_functions;
};

void I2CDev_Init(I2CDev* p_this, const I2cFunctions* i2c_functions);
void I2CDev_Write(I2CDev* p_this, const I2cRequest* request);
void I2CDev_Read(I2CDev* p_this, const I2cRequest* request);

#endif  // ROBOT_CORE_I2C_DEV_H_
