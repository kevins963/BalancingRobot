#ifndef ROBOT_CORE_BUFFERED_IO_H_
#define ROBOT_CORE_BUFFERED_IO_H_

#include "robot/core/circle_buffer.h"
#include "robot/core/util.h"

// public defines
FORWARD_DECLARE_STRUCT(BufferedIoEvent)
FORWARD_DECLARE_STRUCT(BufferedIo)

// public structs
struct BufferedIoEvent {
  int (*EventWrite)(const CircleBuffer* buffer);
};

struct BufferedIo {
  BufferedIoEvent event;
  CircleBuffer rx;
  CircleBuffer tx;
};

DEFINE_EMPTY_STRUCT(BufferedIoEvent)
DEFINE_EMPTY_STRUCT(BufferedIo)

// public functions
void BufferedIo_Init(BufferedIo* p_this, const BufferedIoEvent* events,
                     uint8_t* rx_buf, int rx_size, uint8_t* tx_buf,
                     int tx_size);

void BufferedIo_Run(BufferedIo* p_this);

int BufferedIo_Write(BufferedIo* p_this, const uint8_t* buffer, int offset,
                     int write_count);
int BufferedIo_Read(BufferedIo* p_this, const uint8_t* buffer, int offset,
                    int read_count);

#endif  // ROBOT_CORE_BUFFERED_IO_H_
