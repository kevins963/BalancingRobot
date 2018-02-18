#include "robot/core/core.h"

#include <stdio.h>
#include <string.h>

#include "robot/core/integer.h"

CoreApp core_app_;

void CoreApp_Init(CoreAppDrivers* drivers) {
  core_app_ = kEmptyCoreApp;
  core_app_.drivers = *drivers;
}
void CoreApp_Run() {
  static uint8_t buf[64];

  {
    static int tick = 0;
    if (tick < SystemTime_GetTimeMsec(core_app_.drivers.system_time)) {
      tick = SystemTime_GetTimeMsec(core_app_.drivers.system_time) + 10;
      Encoder_Run(core_app_.drivers.encoder_m1);
      Encoder_Run(core_app_.drivers.encoder_m2);
      DebugComm_Run(core_app_.drivers.debug_comm);
    }
  }
  {
    static int last_sys_tick = 0;
    if (last_sys_tick < SystemTime_GetTimeMsec(core_app_.drivers.system_time)) {
      last_sys_tick =
          SystemTime_GetTimeMsec(core_app_.drivers.system_time) + 1000;
      sprintf((char *)buf, "M1: %ld, M2 %ld\r\n",
                Encoder_GetPosition(core_app_.drivers.encoder_m1),
                Encoder_GetPosition(core_app_.drivers.encoder_m2));
      DebugComm_SendData(core_app_.drivers.debug_comm, buf, strlen((char const*)buf));
    }
  }
}
