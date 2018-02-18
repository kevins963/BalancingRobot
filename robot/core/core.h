#ifndef ROBOT_CORE_CORE_H
#define ROBOT_CORE_CORE_H

#include "robot/core/debug_comm.h"
#include "robot/core/encoder.h"
#include "robot/core/system_time.h"

// public defines
FORWARD_DECLARE_STRUCT(CoreAppDrivers)
FORWARD_DECLARE_STRUCT(CoreApp)

// public structs
struct CoreAppDrivers {
  Encoder* encoder_m1;
  Encoder* encoder_m2;
  DebugComm* debug_comm;
  SystemTime* system_time;
};

struct CoreApp {
  CoreAppDrivers drivers;
};

DEFINE_EMPTY_STRUCT(CoreAppDrivers)
DEFINE_EMPTY_STRUCT(CoreApp)

void CoreApp_Init(CoreAppDrivers* drivers);
void CoreApp_Run();

#endif  // ROBOT_CORE_CORE_H
