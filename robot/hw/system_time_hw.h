#ifndef ROBOT_HW_SYSTEM_TIME_HW_H
#define ROBOT_HW_SYSTEM_TIME_HW_H

#include "robot/core/system_time.h"
#include "robot/core/util.h"

// public defines
FORWARD_DECLARE_STRUCT(SystemTimeHw)

// public structs
struct SystemTimeHw {
  SystemTime base;
};

DEFINE_EMPTY_STRUCT(SystemTimeHw)

// public functions
void SystemTimeHw_Init(SystemTimeHw* p_this);

#endif  // ROBOT_HW_SYSTEM_TIME_HW_H
