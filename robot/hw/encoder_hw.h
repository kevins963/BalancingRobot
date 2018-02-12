#ifndef ROBOT_HW_ENCODER_HW_H
#define ROBOT_HW_ENCODER_HW_H

#include "robot/core/encoder.h"
#include "robot/core/util.h"

// public defines
typedef enum { MotorTypes_Motor1, MotorTypes_Motor2 } MotorTypes;

// public structs
FORWARD_DECLARE_STRUCT(EncoderHw)

struct EncoderHw {
  Encoder base;
  MotorTypes motor_type;
};

DEFINE_EMPTY_STRUCT(EncoderHw)

// public functions
void EncoderHw_Init(EncoderHw* p_this, MotorTypes motor_type);

#endif  // ROBOT_HW_ENCODER_HW_H
