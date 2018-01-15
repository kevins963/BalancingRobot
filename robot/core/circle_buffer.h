#ifndef ROBOT_CORE_CIRCLE_BUFFER_H
#define ROBOT_CORE_CIRCLE_BUFFER_H

#include <stdint.h>

// public defines

// public structs
typedef struct  {
  uint8_t* buffer;
  int write_pos;
  int read_pos;
  // using count to allow for full buffer size
  int count;
  int size;
} CircleBuffer;

// public functions
void CircleBuffer_Init(CircleBuffer* p_this, uint_t* buffer, int size);

uint8_t CircleBuffer_Read(CircleBuffer* p_this);
bool CircleBuffer_Write(CircleBuffer* p_this, uint8_t data);

#endif  // ROBOT_CORE_CIRCLE_BUFFER_H