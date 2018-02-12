#ifndef ROBOT_CORE_DEBUG_COMM_H_
#define ROBOT_CORE_DEBUG_COMM_H_

#include "robot/core/buffered_io.h"
#include "robot/core/integer.h"
#include "robot/core/util.h"

// public defines
#define kDebugCommRxSize 512
#define kDebugCommTxSize 512

FORWARD_DECLARE_STRUCT(DebugComm);

// public structs
struct DebugComm {
  uint8_t rx[kDebugCommRxSize];
  uint8_t tx[kDebugCommTxSize];
  BufferedIo buffered_io;
};

DEFINE_EMPTY_STRUCT(DebugComm);

// public functions
void DebugComm_Init(DebugComm* p_this,
                    const BufferedIoEvent* buffered_io_events);
void DebugComm_Run(DebugComm* p_this);
int DebugComm_SendData(DebugComm* p_this, const uint8_t* buffer, int count);

#endif  // ROBOT_CORE_DEBUG_COMM_H_
