#include "robot/core/circle_buffer.h"
// private defines

// private structs

// private functions
int CircleBuffer_GetRemainingCount(CircleBuffer* p_this);

int CircleBuffer_GetRemainingCount(CircleBuffer* p_this) {
  // size will always be 1 short of full
  return (p_this->size - CircleBuffer_GetCount(p_this)) - 1;
}

void CircleBuffer_IncrementRead(CircleBuffer* p_this, int count) {
  p_this->read_pos = (p_this->read_pos + count) % p_this->size;
}

void CircleBuffer_IncrementWrite(CircleBuffer* p_this, int count) {
  p_this->write_pos = (p_this->write_pos + count) % p_this->size;
}

// public functions
void CircleBuffer_Init(CircleBuffer* p_this, uint8_t* buffer, int size) {
  p_this->buffer = buffer;
  p_this->size = size;
  p_this->read_pos = 0;
  p_this->write_pos = 0;
}

uint8_t CircleBuffer_ReadByte(CircleBuffer* p_this) {
  uint8_t data = p_this->buffer[p_this->read_pos];

  if (CircleBuffer_GetCount(p_this) > 0) {
    CircleBuffer_IncrementRead(p_this, 1);
  }

  return data;
}

uint8_t CircleBuffer_Peek(const CircleBuffer* p_this, int offset) {
  int read_index = (p_this->read_pos + offset) % p_this->size;
  return p_this->buffer[read_index];
}

bool CircleBuffer_WriteByte(CircleBuffer* p_this, uint8_t data) {
  if (CircleBuffer_GetRemainingCount(p_this) > 0) {
    p_this->buffer[p_this->write_pos] = data;
    CircleBuffer_IncrementWrite(p_this, 1);
    return true;
  }
  return false;
}

int CircleBuffer_WriteBlock(CircleBuffer* p_this, const uint8_t* data,
                            int count) {
  int write_size = MIN(CircleBuffer_GetRemainingCount(p_this), count);

  // TODO(kevins963): slow, move to memcpy
  for (int i = 0; i < write_size; i++) {
    CircleBuffer_WriteByte(p_this, data[i]);
  }

  return write_size;
}

int CircleBuffer_CopyFrom(CircleBuffer* p_this, const CircleBuffer* src) {
  int count =
      MIN(CircleBuffer_GetRemainingCount(p_this), CircleBuffer_GetCount(src));
  int copy_size = 0;

  // TODO(kevins963): slow, move to memcpy
  for (int i = 0; i < count; i++) {
    copy_size += CircleBuffer_WriteByte(p_this, CircleBuffer_Peek(src, i));
  }

  return copy_size;
}

int CircleBuffer_GetCount(const CircleBuffer* p_this) {
  return (p_this->write_pos < p_this->read_pos)
             ? (p_this->write_pos - p_this->read_pos + p_this->size)
             : (p_this->write_pos - p_this->read_pos);
}
