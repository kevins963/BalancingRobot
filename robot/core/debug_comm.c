#include "robot/core/debug_comm.h"

#include "robot/core/buffered_io.h"

// private defines

// private structs

// public functions
void DebugComm_Init(DebugComm* p_this,
                    const BufferedIoEvent* buffered_io_events) {
  InitClass(DebugComm);
  BufferedIo_Init(&p_this->buffered_io, buffered_io_events, p_this->rx,
                  kDebugCommRxSize, p_this->tx, kDebugCommTxSize);
}

void DebugComm_Run(DebugComm* p_this) { BufferedIo_Run(&p_this->buffered_io); }

int DebugComm_SendData(DebugComm* p_this, const uint8_t* buffer, int count) {
  return BufferedIo_Write(&p_this->buffered_io, buffer, 0, count);
}

// private functions
