#ifndef ROBOT_CORE_L3GD20H_H_
#define ROBOT_CORE_L3GD20H_H_

#include "robot/core/i2c_dev.h"
#include "robot/core/integer.h"
#include "robot/core/util.h"

#define L3GD20H_ADDRESS 0xD6
#define L3GD20H_DEFAULT_ADDRESS 0xD6

#define L3GD20H_WHO_AM_I_ADR 0xD7

#define L3GD20H_RA_WHO_AM_I 0x0F
#define L3GD20H_RA_CTRL1 0x20
#define L3GD20H_RA_CTRL2 0x21
#define L3GD20H_RA_CTRL3 0x22
#define L3GD20H_RA_CTRL4 0x23
#define L3GD20H_RA_CTRL5 0x24
#define L3GD20H_RA_REFERENCE 0x25
#define L3GD20H_RA_OUT_TEMP 0x26
#define L3GD20H_RA_STATUS 0x27
#define L3GD20H_RA_OUT_X_L 0x28
#define L3GD20H_RA_OUT_X_H 0x29
#define L3GD20H_RA_OUT_Y_L 0x2A
#define L3GD20H_RA_OUT_Y_H 0x2B
#define L3GD20H_RA_OUT_Z_L 0x2C
#define L3GD20H_RA_OUT_Z_H 0x2D
#define L3GD20H_RA_FIFO_CTRL 0x2E
#define L3GD20H_RA_FIFO_SRC 0x2F
#define L3GD20H_RA_IG_CFG 0x30
#define L3GD20H_RA_IG_SRC 0x31
#define L3GD20H_RA_IG_THS_XH 0x32
#define L3GD20H_RA_IG_THS_XL 0X33
#define L3GD20H_RA_IG_THS_YH 0X34
#define L3GD20H_RA_IG_THS_YL 0x35
#define L3GD20H_RA_IG_THS_ZH 0X36
#define L3GD20H_RA_IG_THS_ZL 0x37
#define L3GD20H_RA_IG_DURATION 0X38
#define L3GD20H_RA_LOW_ODR 0x39

#define L3GD20H_ODR_BIT 6
#define L3GD20H_BW_BIT 4
#define L3GD20H_PD_BIT 3
#define L3GD20H_ZEN_BIT 2
#define L3GD20H_YEN_BIT 1
#define L3GD20H_XEN_BIT 0

#define L3GD20H_CTRL1_XEN (1 << L3GD20H_XEN_BIT)
#define L3GD20H_CTRL1_YEN (1 << L3GD20H_YEN_BIT)
#define L3GD20H_CTRL1_ZEN (1 << L3GD20H_ZEN_BIT)

#define L3GD20H_CTRL1_RATE_100_12 (0x00 << L3GD20H_ODR_BIT)
#define L3GD20H_CTRL1_RATE_200_25 (0x01 << L3GD20H_ODR_BIT)
#define L3GD20H_CTRL1_RATE_400_50 (0x02 << L3GD20H_ODR_BIT)
#define L3GD20H_CTRL1_RATE_800_50 (0x03 << L3GD20H_ODR_BIT)

#define L3GD20H_CTRL1_BW_LOW (0x00 << L3GD20H_BW_BIT)
#define L3GD20H_CTRL1_BW_MED_LOW (0x01 << L3GD20H_BW_BIT)
#define L3GD20H_CTRL1_BW_MED_HIGH (0x02 << L3GD20H_BW_BIT)
#define L3GD20H_CTRL1_BW_HIGH (0x03 << L3GD20H_BW_BIT)

#define L3GD20H_CTRL1_PD_NORMAL (1 << L3GD20H_PD_BIT)

#define L3GD20H_CTRL2_HPM_HRF 0  // this resets on reading REFERENCE
#define L3GD20H_CTRL2_HPM_REFERENCE (1 << 4)
#define L3GD20H_CTRL2_HPM_NORMAL (2 << 4)
#define L3GD20H_CTRL2_HPM_AUTORESET (3 << 4)
#define L3GD20H_CTRL2_HPM_MASK (3 << 4)

#define L3GD20H_CTRL2_HPCF1 0
#define L3GD20H_CTRL2_HPCF2 1
#define L3GD20H_CTRL2_HPCF3 2
#define L3GD20H_CTRL2_HPCF4 3
#define L3GD20H_CTRL2_HPCF5 4
#define L3GD20H_CTRL2_HPCF6 5
#define L3GD20H_CTRL2_HPCF7 6
#define L3GD20H_CTRL2_HPCF8 7
#define L3GD20H_CTRL2_HPCF9 8
#define L3GD20H_CTRL2_HPCF10 9
#define L3GD20H_CTRL2_HPCF_MASK 0xF

#define L3GD20H_CTRL3_INT1_IG_BIT (1 << 7)
#define L3GD20H_CTRL3_INT1_BOOT_BIT (1 << 6)
#define L3GD20H_CTRL3_H_LACTIVE_BIT (1 << 5)
#define L3GD20H_CTRL3_PP_OD_BIT (1 << 4)
#define L3GD20H_CTRL3_INT2_DRDY_BIT (1 << 3)
#define L3GD20H_CTRL3_INT2_FTH_BIT (1 << 2)
#define L3GD20H_CTRL3_INT2_ORUN_BIT (1 << 1)
#define L3GD20H_CTRL3_INT2_EMPTY_BIT (1 << 0)

#define L3GD20H_CTRL3_PUSH_PULL 0
#define L3GD20H_CTRL3_OPEN_DRAIN (1 << 4)

#define L3GD20H_CTRL4_BDU_CONTINIOUS 0
#define L3GD20H_CTRL4_BDU_WAIT (1 << 7)

#define L3GD20H_CTRL4_BLE_BIG 0
#define L3GD20H_CTRL4_BLE_LITTLE (1 << 6)
#define L3GD20H_CTRL4_IMPEN_LATCH_EN (1 << 3)
#define L3GD20H_CTRL4_ST_BIT 2
#define L3GD20H_CTRL4_ST_LENGTH 2
#define L3GD20H_CTRL4_SIM_BIT 0

#define L3GD20H_CTRL4_BIG_ENDIAN 1
#define L3GD20H_CTRL4_LITTLE_ENDIAN 0

#define L3GD20H_CTRL4_FS_250 (0x00 << 4)
#define L3GD20H_CTRL4_FS_500 (0x01 << 4)
#define L3GD20H_CTRL4_FS_2000 (0x02 << 4)
#define L3GD20H_CTRL4_FS_MASK (0x03 << 4)

#define L3GD20H_SENSITIVITY_250DPS (0.00875F)
#define L3GD20H_SENSITIVITY_500DPS (0.01750F)
#define L3GD20H_SENSITIVITY_2000DPS (0.0700F)

#define L3GD20H_CTRL4_SELF_TEST_NORMAL (0x00 << 1)
#define L3GD20H_CTRL4_SELF_TEST_0 (0x01 << 1)
#define L3GD20H_CTRL4_SELF_TEST_1 (0x03 << 1)

#define L3GD20H_CTRL4_SPI_4_WIRE 0
#define L3GD20H_CTRL4_SPI_3_WIRE 1

#define L3GD20H_CTRL5_BOOT_REBOOT (1 << 7)
#define L3GD20H_CTRL5_FIFO_EN (1 << 6)
#define L3GD20H_CTRL5_STOPONFTH (1 << 5)
#define L3GD20H_CTRL5_HPF_EN (1 << 4)
#define L3GD20H_CTRL5_IG_SEL_BIT 3
#define L3GD20H_CTRL5_IG_SEL_LENGTH 2
#define L3GD20H_CTRL5_OUT_SEL_BIT 1
#define L3GD20H_CTRL5_OUT_SEL_LENGTH 2

#define L3GD20H_CTRL5_INT_NON_HIGH_PASS (0x00 << 2)
#define L3GD20H_CTRL5_INT_HIGH_PASS (0x01 << 2)
#define L3GD20H_CTRL5_INT_LOW_PASS \
  (0x02 << 2)  // depends on HPEN                  T_
#define L3GD20H_CTRL5_INT_LOW_HIGH_PASS (0x03 << 2)  // depends on HPEN

#define L3GD20H_CTRL5_OUT_NON_HIGH_PASS (0x00)
#define L3GD20H_CTRL5_OUT_HIGH_PASS (0x01)
#define L3GD20H_CTRL5_OUT_LOW_PASS (0x02)       // depends on HPEN T_
#define L3GD20H_CTRL5_OUT_LOW_HIGH_PASS (0x03)  // depends on HPEN

#define L3GD20H_ZYXOR_BIT 7
#define L3GD20H_ZOR_BIT 6
#define L3GD20H_YOR_BIT 5
#define L3GD20H_XOR_BIT 4
#define L3GD20H_ZYXDA_BIT 3
#define L3GD20H_ZDA_BIT 2
#define L3GD20H_YDA_BIT 1
#define L3GD20H_XDA_BIT 0

#define L3GD20H_CTRLFIFO_MODE_BIT 7
#define L3GD20H_CTRLFIFO_MODE_LENGTH 3
#define L3GD20H_CTRLFIFO_TH_BIT 4
#define L3GD20H_CTRLFIFO_TH_LENGTH 5

#define L3GD20H_CTRLFIFO_FM_BYPASS (0x00 << 5)
#define L3GD20H_CTRLFIFO_FM_FIFO (0x01 << 5)
#define L3GD20H_CTRLFIFO_FM_STREAM (0x02 << 5)
#define L3GD20H_CTRLFIFO_FM_STREAM_FIFO (0x03 << 5)
#define L3GD20H_CTRLFIFO_FM_BYPASS_STREAM (0x04 << 5)
#define L3GD20H_CTRLFIFO_FM_DYNAMIC_STREAM (0x06 << 5)
#define L3GD20H_CTRLFIFO_FM_BYPASS_FIFO (0x07 << 5)

#define L3GD20H_FIFO_TH_STATUS_BIT 7
#define L3GD20H_OVRN_BIT 6
#define L3GD20H_EMPTY_BIT 5
#define L3GD20H_FIFO_FSS_BIT 4
#define L3GD20H_FIFO_FSS_LENGTH 5

#define L3GD20H_AND_OR_BIT 7
#define L3GD20H_LIR_BIT 6
#define L3GD20H_ZHIE_BIT 5
#define L3GD20H_ZLIE_BIT 4
#define L3GD20H_YHIE_BIT 3
#define L3GD20H_YLIE_BIT 2
#define L3GD20H_XHIE_BIT 1
#define L3GD20H_XLIE_BIT 0

#define L3GD20H_AND_OR_OR 0
#define L3GD20H_AND_OR_AND 1

#define L3GD20H_IA_BIT 6
#define L3GD20H_ZH_BIT 5
#define L3GD20H_ZL_BIT 4
#define L3GD20H_YH_BIT 3
#define L3GD20H_YL_BIT 2
#define L3GD20H_XH_BIT 1
#define L3GD20H_XL_BIT 0

#define L3GD20H_DCRM_BIT 7

#define L3GD20H_DCRM_RESET 0
#define L3GD20H_DCRM_DEC 1

#define L3GD20H_WAIT_BIT 7
#define L3GD20H_DUR_BIT 6
#define L3GD20H_DUR_LENGTH 7

#define L3GD20H_LOW_ODR_BIT 0
#define L3GD20H_SW_RESET_BIT 2
#define L3GD20H_I2C_DIS_BIT 3
#define L3GD20H_DRDY_HL_BIT 5

#define L3GD20H_REGISTERS (L3GD20H_RA_LOW_ODR + 1)

FORWARD_DECLARE_STRUCT(L3GD20H);
typedef enum L3GD20HCommands L3GD20HCommands;

enum L3GD20HCommands {
  L3GD20HCommands_Init,
  L3GD20HCommands_Generic,
  L3GD20HCommands_ReadGyro,
};

BEGIN_STRUCT(GyroData)
float x;
float y;
float z;
END_STRUCT(GyroData)
DEFINE_EMPTY_STRUCT(I2CDev)

struct L3GD20H {
  I2CDev i2c_dev;
  uint8_t registers[L3GD20H_REGISTERS];
  uint8_t write_registers[L3GD20H_REGISTERS];
  uint8_t device_address;
  uint8_t who_am_i;
  L3GD20HCommands last_command;
  GyroData gyro_data;
};

DEFINE_EMPTY_STRUCT(L3GD20H)

void L3GD20H_Init(L3GD20H* p_this, const I2cFunctions* i2c_functions);
void L3GD20H_Start(L3GD20H* p_this);
void L3GD20H_ReadAll(L3GD20H* p_this);
void L3GD20H_ReadWhoAmI(L3GD20H* p_this);
void L3GD20H_ReadGyroData(L3GD20H* p_this);
void L3GD20H_OnComplete(L3GD20H* p_this);

#endif  // ROBOT_CORE_L3GD20H_H_
