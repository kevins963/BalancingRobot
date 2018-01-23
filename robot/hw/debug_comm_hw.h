#ifndef ROBOT_HW_DEBUG_COMM_HW_H_
#define ROBOT_HW_DEBUG_COMM_HW_H_

#include "robot/core/integer.h"

// public defines

// public structs

// public functions
void DebugCommHw_Init(void);
void DebugCommHw_Write(const uint8_t* buf, int count);

#endif  // ROBOT_HW_DEBUG_COMM_HW_H_
