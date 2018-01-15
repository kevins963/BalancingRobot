#include "robot/core/circle_buffer.h"
// private defines

// private structs
int CircleBuffer_GetCount(CircleBuffer* p_this) { return p_this->count; }

// public functions
void CircleBuffer_Init(CircleBuffer* p_this, uint_t* buffer, int size) {
  p_this->buffer = buffer;
  p_this->size = size;
  p_this->read_pos = 0;
  p_this->write_pos = 0;
  p_this->count = 0;
}

// private functions