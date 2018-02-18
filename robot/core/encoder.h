#ifndef ROBOT_CORE_ENCODER_H_
#define ROBOT_CORE_ENCODER_H_

#include "robot/core/defs.h"
#include "robot/core/integer.h"

// public defines
typedef struct Encoder Encoder;
typedef struct EncoderFunctions EncoderFunctions;

// public structs
struct EncoderFunctions {
  uint16_t (*GetEncoderCount)(Encoder* p_this);
};

struct Encoder {
  EncoderFunctions v_functions;

  int32_t last_encoder_count;
  int32_t current_encoder_count;
  int32_t encoder_position;
};

// public functions
void Encoder_Init(Encoder* p_this);

void Encoder_Run(Encoder* p_this);

int32_t Encoder_GetPosition(Encoder* p_this);

#endif  // ROBOT_CORE_ENCODER_H_
