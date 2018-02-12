#ifndef ROBOT_CORE_CIRCLE_BUFFER_H
#define ROBOT_CORE_CIRCLE_BUFFER_H

#include "robot/core/integer.h"

// public defines

// public structs
typedef struct {
  uint8_t* buffer;
  int write_pos;
  int read_pos;
  int size;
} CircleBuffer;

// public functions
void CircleBuffer_Init(CircleBuffer* p_this, uint8_t* buffer, int size);
uint8_t CircleBuffer_ReadByte(CircleBuffer* p_this);
uint8_t CircleBuffer_Peek(const CircleBuffer* p_this, int offset);
bool CircleBuffer_WriteByte(CircleBuffer* p_this, uint8_t data);
int CircleBuffer_WriteBlock(CircleBuffer* p_this, const uint8_t* data,
                            int count);
int CircleBuffer_CopyFrom(CircleBuffer* p_this, const CircleBuffer* src);
int CircleBuffer_GetCount(const CircleBuffer* p_this);
void CircleBuffer_IncrementRead(CircleBuffer* p_this, int count);
void CircleBuffer_IncrementWrite(CircleBuffer* p_this, int count);
#endif  // ROBOT_CORE_CIRCLE_BUFFER_H
