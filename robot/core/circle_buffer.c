#include "robot/core/circle_buffer.h"
// private defines

// private structs

// private functions
int CircleBuffer_GetCount(CircleBuffer* p_this) {
  return (p_this->write_pos < p_this->read_pos)
             ? (p_this->write_pos - p_this->read_pos + p_this->size)
             : (p_this->write_pos - p_this->read_pos);
}

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

bool CircleBuffer_WriteByte(CircleBuffer* p_this, uint8_t data) {
  if (CircleBuffer_GetRemainingCount(p_this) > 0) {
    p_this->buffer[p_this->write_pos] = data;
    CircleBuffer_IncrementWrite(p_this, 1);
    return true;
  }
  return false;
}
