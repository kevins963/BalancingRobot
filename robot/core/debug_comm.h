#ifndef ROBOT_CORE_DEBUG_COMM_H
#define ROBOT_CORE_DEBUG_COMM_H

// public defines

// public structs
typedef struct  {
  uint8_t rx_buf[];
  uint8_t tx_buf[];
} DebugComm;

// public functions
void DebugComm_Init(void);

#endif  // ROBOT_CORE_DEBUG_COMM_H