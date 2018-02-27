#ifndef ROBOT_CORE_MEMS_HW_H_
#define ROBOT_CORE_MEMS_HW_H_

#include "robot/core/l3gd20h.h"
#include "robot/core/lsm303d.h"
#include "robot/core/util.h"
#include "third_party/stm32f4/drivers/cmsis/device/st/stm32f4xx/include/stm32f4xx.h"

BEGIN_STRUCT(MemsHw)
L3GD20H gyro_driver;
LSM303D accel_driver;
I2C_HandleTypeDef i2c_handle;
END_STRUCT(MemsHw)

void MemsHw_Init(MemsHw* p_this);
#endif  // ROBOT_CORE_MEMS_HW_H_
