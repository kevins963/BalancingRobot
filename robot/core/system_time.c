#include "robot/core/system_time.h"

void SystemTime_Init(SystemTime* p_this, SystemTimeFunctions functions) {
  InitClass(SystemTime);
  p_this->v_functions = functions;
}

uint32_t SystemTime_GetTimeMsec(SystemTime* p_this) {
  return p_this->v_functions.GetMsecTick();
}
