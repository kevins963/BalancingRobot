#include <assert.h>

#include "robot/core/encoder.h"

static const Encoder kEmptyEncoder;

uint16_t Encoder_GetEncoderCount(Encoder* p_this) {
  assert(p_this->v_functions.GetEncoderCount);
  return p_this->v_functions.GetEncoderCount(p_this);
}

void Encoder_UpdateEncoderPosition(Encoder* p_this) {
  uint32_t encoder_count = Encoder_GetEncoderCount(p_this);
  int32_t encoder_diff = (int32_t)(encoder_count - p_this->last_encoder_count);

  p_this->encoder_position += encoder_diff;
  p_this->last_encoder_count = encoder_count;
}

void Encoder_Init(Encoder* p_this) { *p_this = kEmptyEncoder; }

void Encoder_Run(Encoder* p_this) { Encoder_UpdateEncoderPosition(p_this); }

int32_t Encoder_GetPosition(Encoder* p_this) {
  return p_this->encoder_position;
}