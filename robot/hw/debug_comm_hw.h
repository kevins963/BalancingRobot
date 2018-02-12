#ifndef ROBOT_HW_DEBUG_COMM_HW_H_
#define ROBOT_HW_DEBUG_COMM_HW_H_

#include "robot/core/debug_comm.h"
#include "robot/core/integer.h"

// public defines
FORWARD_DECLARE_STRUCT(DebugCommHw)

// public structs
struct DebugCommHw {
  DebugComm base;
};

// public functions
void DebugCommHw_Init(DebugCommHw* p_this);

#endif  // ROBOT_HW_DEBUG_COMM_HW_H_
