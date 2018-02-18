#ifndef ROBOT_CORE_SYSTEM_TIME_H
#define ROBOT_CORE_SYSTEM_TIME_H

#include "robot/core/integer.h"
#include "robot/core/util.h"

// public defines
FORWARD_DECLARE_STRUCT(SystemTimeFunctions)
FORWARD_DECLARE_STRUCT(SystemTime)

// public structs
struct SystemTimeFunctions {
  uint32_t (*GetMsecTick)(void);
};

struct SystemTime {
  SystemTimeFunctions v_functions;
};

DEFINE_EMPTY_STRUCT(SystemTimeFunctions)
DEFINE_EMPTY_STRUCT(SystemTime)

// public functions
void SystemTime_Init(SystemTime* p_this, SystemTimeFunctions functions);
uint32_t SystemTime_GetTimeMsec(SystemTime* p_this);

#endif  // ROBOT_CORE_SYSTEM_TIME_H
