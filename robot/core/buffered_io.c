#include "robot/core/buffered_io.h"

#include <assert.h>

// private defines

// private structs

// private function declarations
void BufferedIo_EventWrite(BufferedIo* p_this) {
  assert(p_this->event.EventWrite);
  int write_size = p_this->event.EventWrite(&p_this->tx);
  CircleBuffer_IncrementRead(&p_this->tx, write_size);
}

// public functions
void BufferedIo_Init(BufferedIo* p_this, const BufferedIoEvent* events,
                     uint8_t* rx_buf, int rx_size, uint8_t* tx_buf,
                     int tx_size) {
  InitClass(BufferedIo);
  p_this->event = *events;
  CircleBuffer_Init(&p_this->rx, rx_buf, rx_size);
  CircleBuffer_Init(&p_this->tx, tx_buf, tx_size);
}

void BufferedIo_Run(BufferedIo* p_this) {
  if (CircleBuffer_GetCount(&p_this->tx) > 0) {
    BufferedIo_EventWrite(p_this);
  }
}
// Write will write to the buffer io and attempt to push data to the io handler,
int BufferedIo_Write(BufferedIo* p_this, const uint8_t* buffer, int offset,
                     int write_count) {
  int write_size =
      CircleBuffer_WriteBlock(&p_this->tx, buffer + offset, write_count);
  BufferedIo_EventWrite(p_this);
  return write_size;
}

int BufferedIo_Read(BufferedIo* p_this, const uint8_t* buffer, int offset,
                    int read_count) {
  return CircleBuffer_WriteBlock(&p_this->rx, buffer + offset, read_count);
}

// private functions
