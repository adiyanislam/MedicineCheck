#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MPU6050_ADDR 0x68
#define INT_PIN 14 // Use the GPIO pin connected to MPU6050's INT pin
#define MPU6050_I2CADDR_DEFAULT                                                \
  0x68                         ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID 0x68 ///< The correct MPU6050_WHO_AM_I value

#define MPU60X0_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU60X0_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU60X0_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU60X0_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU60X0_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU60X0_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU60X0_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU60X0_RA_XA_OFFS_L_TC     0x07
#define MPU60X0_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU60X0_RA_YA_OFFS_L_TC     0x09
#define MPU60X0_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU60X0_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU60X0_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU60X0_RA_XG_OFFS_USRL     0x14
#define MPU60X0_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU60X0_RA_YG_OFFS_USRL     0x16
#define MPU60X0_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU60X0_RA_ZG_OFFS_USRL     0x18
#define MPU60X0_RA_SMPLRT_DIV       0x19
#define MPU60X0_RA_CONFIG           0x1A
#define MPU60X0_RA_GYRO_CONFIG      0x1B
#define MPU60X0_RA_ACCEL_CONFIG     0x1C
#define MPU60X0_RA_FF_THR           0x1D
#define MPU60X0_RA_FF_DUR           0x1E
#define MPU60X0_RA_MOT_THR          0x1F
#define MPU60X0_RA_MOT_DUR          0x20
#define MPU60X0_RA_ZRMOT_THR        0x21
#define MPU60X0_RA_ZRMOT_DUR        0x22
#define MPU60X0_RA_FIFO_EN          0x23
#define MPU60X0_RA_I2C_MST_CTRL     0x24
#define MPU60X0_RA_I2C_SLV0_ADDR    0x25
#define MPU60X0_RA_I2C_SLV0_REG     0x26
#define MPU60X0_RA_I2C_SLV0_CTRL    0x27
#define MPU60X0_RA_I2C_SLV1_ADDR    0x28
#define MPU60X0_RA_I2C_SLV1_REG     0x29
#define MPU60X0_RA_I2C_SLV1_CTRL    0x2A
#define MPU60X0_RA_I2C_SLV2_ADDR    0x2B
#define MPU60X0_RA_I2C_SLV2_REG     0x2C
#define MPU60X0_RA_I2C_SLV2_CTRL    0x2D
#define MPU60X0_RA_I2C_SLV3_ADDR    0x2E
#define MPU60X0_RA_I2C_SLV3_REG     0x2F
#define MPU60X0_RA_I2C_SLV3_CTRL    0x30
#define MPU60X0_RA_I2C_SLV4_ADDR    0x31
#define MPU60X0_RA_I2C_SLV4_REG     0x32
#define MPU60X0_RA_I2C_SLV4_DO      0x33
#define MPU60X0_RA_I2C_SLV4_CTRL    0x34
#define MPU60X0_RA_I2C_SLV4_DI      0x35
#define MPU60X0_RA_I2C_MST_STATUS   0x36
#define MPU60X0_RA_INT_PIN_CFG      0x37
#define MPU60X0_RA_INT_ENABLE       0x38
#define MPU60X0_RA_DMP_INT_STATUS   0x39
#define MPU60X0_RA_INT_STATUS       0x3A
#define MPU60X0_RA_ACCEL_XOUT_H     0x3B
#define MPU60X0_RA_ACCEL_XOUT_L     0x3C
#define MPU60X0_RA_ACCEL_YOUT_H     0x3D
#define MPU60X0_RA_ACCEL_YOUT_L     0x3E
#define MPU60X0_RA_ACCEL_ZOUT_H     0x3F
#define MPU60X0_RA_ACCEL_ZOUT_L     0x40
#define MPU60X0_RA_TEMP_OUT_H       0x41
#define MPU60X0_RA_TEMP_OUT_L       0x42
#define MPU60X0_RA_GYRO_XOUT_H      0x43
#define MPU60X0_RA_GYRO_XOUT_L      0x44
#define MPU60X0_RA_GYRO_YOUT_H      0x45
#define MPU60X0_RA_GYRO_YOUT_L      0x46
#define MPU60X0_RA_GYRO_ZOUT_H      0x47
#define MPU60X0_RA_GYRO_ZOUT_L      0x48
#define MPU60X0_RA_EXT_SENS_DATA_00 0x49
#define MPU60X0_RA_EXT_SENS_DATA_01 0x4A
#define MPU60X0_RA_EXT_SENS_DATA_02 0x4B
#define MPU60X0_RA_EXT_SENS_DATA_03 0x4C
#define MPU60X0_RA_EXT_SENS_DATA_04 0x4D
#define MPU60X0_RA_EXT_SENS_DATA_05 0x4E
#define MPU60X0_RA_EXT_SENS_DATA_06 0x4F
#define MPU60X0_RA_EXT_SENS_DATA_07 0x50
#define MPU60X0_RA_EXT_SENS_DATA_08 0x51
#define MPU60X0_RA_EXT_SENS_DATA_09 0x52
#define MPU60X0_RA_EXT_SENS_DATA_10 0x53
#define MPU60X0_RA_EXT_SENS_DATA_11 0x54
#define MPU60X0_RA_EXT_SENS_DATA_12 0x55
#define MPU60X0_RA_EXT_SENS_DATA_13 0x56
#define MPU60X0_RA_EXT_SENS_DATA_14 0x57
#define MPU60X0_RA_EXT_SENS_DATA_15 0x58
#define MPU60X0_RA_EXT_SENS_DATA_16 0x59
#define MPU60X0_RA_EXT_SENS_DATA_17 0x5A
#define MPU60X0_RA_EXT_SENS_DATA_18 0x5B
#define MPU60X0_RA_EXT_SENS_DATA_19 0x5C
#define MPU60X0_RA_EXT_SENS_DATA_20 0x5D
#define MPU60X0_RA_EXT_SENS_DATA_21 0x5E
#define MPU60X0_RA_EXT_SENS_DATA_22 0x5F
#define MPU60X0_RA_EXT_SENS_DATA_23 0x60
#define MPU60X0_RA_MOT_DETECT_STATUS    0x61
#define MPU60X0_RA_I2C_SLV0_DO      0x63
#define MPU60X0_RA_I2C_SLV1_DO      0x64
#define MPU60X0_RA_I2C_SLV2_DO      0x65
#define MPU60X0_RA_I2C_SLV3_DO      0x66
#define MPU60X0_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU60X0_RA_SIGNAL_PATH_RESET    0x68
#define MPU60X0_RA_MOT_DETECT_CTRL      0x69
#define MPU60X0_RA_USER_CTRL        0x6A
#define MPU60X0_RA_PWR_MGMT_1       0x6B
#define MPU60X0_RA_PWR_MGMT_2       0x6C
#define MPU60X0_RA_BANK_SEL         0x6D
#define MPU60X0_RA_MEM_START_ADDR   0x6E
#define MPU60X0_RA_MEM_R_W          0x6F
#define MPU60X0_RA_DMP_CFG_1        0x70
#define MPU60X0_RA_DMP_CFG_2        0x71
#define MPU60X0_RA_FIFO_COUNTH      0x72
#define MPU60X0_RA_FIFO_COUNTL      0x73
#define MPU60X0_RA_FIFO_R_W         0x74
#define MPU60X0_RA_WHO_AM_I         0x75

#define MPU60X0_TC_PWR_MODE_BIT     7
#define MPU60X0_TC_OFFSET_BIT       6
#define MPU60X0_TC_OFFSET_LENGTH    6
#define MPU60X0_TC_OTP_BNK_VLD_BIT  0

#define MPU60X0_VDDIO_LEVEL_VLOGIC  0
#define MPU60X0_VDDIO_LEVEL_VDD     1

#define MPU60X0_CFG_EXT_SYNC_SET_BIT    5
#define MPU60X0_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU60X0_CFG_DLPF_CFG_BIT    2
#define MPU60X0_CFG_DLPF_CFG_LENGTH 3

#define MPU60X0_EXT_SYNC_DISABLED       0x0
#define MPU60X0_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU60X0_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU60X0_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU60X0_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU60X0_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU60X0_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU60X0_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU60X0_DLPF_BW_256         0x00
#define MPU60X0_DLPF_BW_188         0x01
#define MPU60X0_DLPF_BW_98          0x02
#define MPU60X0_DLPF_BW_42          0x03
#define MPU60X0_DLPF_BW_20          0x04
#define MPU60X0_DLPF_BW_10          0x05
#define MPU60X0_DLPF_BW_5           0x06

#define MPU60X0_GCONFIG_FS_SEL_BIT      4
#define MPU60X0_GCONFIG_FS_SEL_LENGTH   2

#define MPU60X0_GYRO_FS_250         0x00
#define MPU60X0_GYRO_FS_500         0x01
#define MPU60X0_GYRO_FS_1000        0x02
#define MPU60X0_GYRO_FS_2000        0x03

#define MPU60X0_ACONFIG_XA_ST_BIT           7
#define MPU60X0_ACONFIG_YA_ST_BIT           6
#define MPU60X0_ACONFIG_ZA_ST_BIT           5
#define MPU60X0_ACONFIG_AFS_SEL_BIT         4
#define MPU60X0_ACONFIG_AFS_SEL_LENGTH      2
#define MPU60X0_ACONFIG_ACCEL_HPF_BIT       2
#define MPU60X0_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU60X0_ACCEL_FS_2          0x00
#define MPU60X0_ACCEL_FS_4          0x01
#define MPU60X0_ACCEL_FS_8          0x02
#define MPU60X0_ACCEL_FS_16         0x03

#define MPU60X0_DHPF_RESET          0x00
#define MPU60X0_DHPF_5              0x01
#define MPU60X0_DHPF_2P5            0x02
#define MPU60X0_DHPF_1P25           0x03
#define MPU60X0_DHPF_0P63           0x04
#define MPU60X0_DHPF_HOLD           0x07

#define MPU60X0_TEMP_FIFO_EN_BIT    7
#define MPU60X0_XG_FIFO_EN_BIT      6
#define MPU60X0_YG_FIFO_EN_BIT      5
#define MPU60X0_ZG_FIFO_EN_BIT      4
#define MPU60X0_ACCEL_FIFO_EN_BIT   3
#define MPU60X0_SLV2_FIFO_EN_BIT    2
#define MPU60X0_SLV1_FIFO_EN_BIT    1
#define MPU60X0_SLV0_FIFO_EN_BIT    0

#define MPU60X0_MULT_MST_EN_BIT     7
#define MPU60X0_WAIT_FOR_ES_BIT     6
#define MPU60X0_SLV_3_FIFO_EN_BIT   5
#define MPU60X0_I2C_MST_P_NSR_BIT   4
#define MPU60X0_I2C_MST_CLK_BIT     3
#define MPU60X0_I2C_MST_CLK_LENGTH  4

#define MPU60X0_CLOCK_DIV_348       0x0
#define MPU60X0_CLOCK_DIV_333       0x1
#define MPU60X0_CLOCK_DIV_320       0x2
#define MPU60X0_CLOCK_DIV_308       0x3
#define MPU60X0_CLOCK_DIV_296       0x4
#define MPU60X0_CLOCK_DIV_286       0x5
#define MPU60X0_CLOCK_DIV_276       0x6
#define MPU60X0_CLOCK_DIV_267       0x7
#define MPU60X0_CLOCK_DIV_258       0x8
#define MPU60X0_CLOCK_DIV_500       0x9
#define MPU60X0_CLOCK_DIV_471       0xA
#define MPU60X0_CLOCK_DIV_444       0xB
#define MPU60X0_CLOCK_DIV_421       0xC
#define MPU60X0_CLOCK_DIV_400       0xD
#define MPU60X0_CLOCK_DIV_381       0xE
#define MPU60X0_CLOCK_DIV_364       0xF

#define MPU60X0_I2C_SLV_RW_BIT      7
#define MPU60X0_I2C_SLV_ADDR_BIT    6
#define MPU60X0_I2C_SLV_ADDR_LENGTH 7
#define MPU60X0_I2C_SLV_EN_BIT      7
#define MPU60X0_I2C_SLV_BYTE_SW_BIT 6
#define MPU60X0_I2C_SLV_REG_DIS_BIT 5
#define MPU60X0_I2C_SLV_GRP_BIT     4
#define MPU60X0_I2C_SLV_LEN_BIT     3
#define MPU60X0_I2C_SLV_LEN_LENGTH  4

#define MPU60X0_I2C_SLV4_RW_BIT         7
#define MPU60X0_I2C_SLV4_ADDR_BIT       6
#define MPU60X0_I2C_SLV4_ADDR_LENGTH    7
#define MPU60X0_I2C_SLV4_EN_BIT         7
#define MPU60X0_I2C_SLV4_INT_EN_BIT     6
#define MPU60X0_I2C_SLV4_REG_DIS_BIT    5
#define MPU60X0_I2C_SLV4_MST_DLY_BIT    4
#define MPU60X0_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU60X0_MST_PASS_THROUGH_BIT    7
#define MPU60X0_MST_I2C_SLV4_DONE_BIT   6
#define MPU60X0_MST_I2C_LOST_ARB_BIT    5
#define MPU60X0_MST_I2C_SLV4_NACK_BIT   4
#define MPU60X0_MST_I2C_SLV3_NACK_BIT   3
#define MPU60X0_MST_I2C_SLV2_NACK_BIT   2
#define MPU60X0_MST_I2C_SLV1_NACK_BIT   1
#define MPU60X0_MST_I2C_SLV0_NACK_BIT   0

#define MPU60X0_INTCFG_INT_LEVEL_BIT        7
#define MPU60X0_INTCFG_INT_OPEN_BIT         6
#define MPU60X0_INTCFG_LATCH_INT_EN_BIT     5
#define MPU60X0_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU60X0_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU60X0_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU60X0_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU60X0_INTCFG_CLKOUT_EN_BIT        0

#define MPU60X0_INTMODE_ACTIVEHIGH  0x00
#define MPU60X0_INTMODE_ACTIVELOW   0x01

#define MPU60X0_INTDRV_PUSHPULL     0x00
#define MPU60X0_INTDRV_OPENDRAIN    0x01

#define MPU60X0_INTLATCH_50USPULSE  0x00
#define MPU60X0_INTLATCH_WAITCLEAR  0x01

#define MPU60X0_INTCLEAR_STATUSREAD 0x00
#define MPU60X0_INTCLEAR_ANYREAD    0x01

#define MPU60X0_INTERRUPT_FF_BIT            7
#define MPU60X0_INTERRUPT_MOT_BIT           6
#define MPU60X0_INTERRUPT_ZMOT_BIT          5
#define MPU60X0_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU60X0_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU60X0_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU60X0_INTERRUPT_DMP_INT_BIT       1
#define MPU60X0_INTERRUPT_DATA_RDY_BIT      0

#define MPU60X0_INTERRUPT_FF                0x80
#define MPU60X0_INTERRUPT_MOT               0x40
#define MPU60X0_INTERRUPT_ZMOT              0x20
#define MPU60X0_INTERRUPT_FIFO_OFLOW        0x10
#define MPU60X0_INTERRUPT_I2C_MST_INT       0x08
#define MPU60X0_INTERRUPT_PLL_RDY_INT       0x04
#define MPU60X0_INTERRUPT_DMP_INT           0x02
#define MPU60X0_INTERRUPT_DATA_RDY          0x01

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU60X0_DMPINT_5_BIT            5
#define MPU60X0_DMPINT_4_BIT            4
#define MPU60X0_DMPINT_3_BIT            3
#define MPU60X0_DMPINT_2_BIT            2
#define MPU60X0_DMPINT_1_BIT            1
#define MPU60X0_DMPINT_0_BIT            0

#define MPU60X0_MOTION_MOT_XNEG_BIT     7
#define MPU60X0_MOTION_MOT_XPOS_BIT     6
#define MPU60X0_MOTION_MOT_YNEG_BIT     5
#define MPU60X0_MOTION_MOT_YPOS_BIT     4
#define MPU60X0_MOTION_MOT_ZNEG_BIT     3
#define MPU60X0_MOTION_MOT_ZPOS_BIT     2
#define MPU60X0_MOTION_MOT_ZRMOT_BIT    0

#define MPU60X0_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU60X0_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU60X0_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU60X0_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU60X0_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU60X0_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU60X0_PATHRESET_GYRO_RESET_BIT    2
#define MPU60X0_PATHRESET_ACCEL_RESET_BIT   1
#define MPU60X0_PATHRESET_TEMP_RESET_BIT    0

#define MPU60X0_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU60X0_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU60X0_DETECT_FF_COUNT_BIT             3
#define MPU60X0_DETECT_FF_COUNT_LENGTH          2
#define MPU60X0_DETECT_MOT_COUNT_BIT            1
#define MPU60X0_DETECT_MOT_COUNT_LENGTH         2

#define MPU60X0_DETECT_DECREMENT_RESET  0x0
#define MPU60X0_DETECT_DECREMENT_1      0x1
#define MPU60X0_DETECT_DECREMENT_2      0x2
#define MPU60X0_DETECT_DECREMENT_4      0x3

#define MPU60X0_USERCTRL_DMP_EN_BIT             7
#define MPU60X0_USERCTRL_FIFO_EN_BIT            6
#define MPU60X0_USERCTRL_I2C_MST_EN_BIT         5
#define MPU60X0_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU60X0_USERCTRL_DMP_RESET_BIT          3
#define MPU60X0_USERCTRL_FIFO_RESET_BIT         2
#define MPU60X0_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU60X0_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU60X0_PWR1_DEVICE_RESET_BIT   7
#define MPU60X0_PWR1_SLEEP_BIT          6
#define MPU60X0_PWR1_CYCLE_BIT          5
#define MPU60X0_PWR1_TEMP_DIS_BIT       3
#define MPU60X0_PWR1_CLKSEL_BIT         2
#define MPU60X0_PWR1_CLKSEL_LENGTH      3

#define MPU60X0_CLOCK_INTERNAL          0x00
#define MPU60X0_CLOCK_PLL_XGYRO         0x01
#define MPU60X0_CLOCK_PLL_YGYRO         0x02
#define MPU60X0_CLOCK_PLL_ZGYRO         0x03
#define MPU60X0_CLOCK_PLL_EXT32K        0x04
#define MPU60X0_CLOCK_PLL_EXT19M        0x05
#define MPU60X0_CLOCK_KEEP_RESET        0x07

#define MPU60X0_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU60X0_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU60X0_PWR2_STBY_XA_BIT            5
#define MPU60X0_PWR2_STBY_YA_BIT            4
#define MPU60X0_PWR2_STBY_ZA_BIT            3
#define MPU60X0_PWR2_STBY_XG_BIT            2
#define MPU60X0_PWR2_STBY_YG_BIT            1
#define MPU60X0_PWR2_STBY_ZG_BIT            0

#define MPU60X0_WAKE_FREQ_1P25      0x0
#define MPU60X0_WAKE_FREQ_2P5       0x1
#define MPU60X0_WAKE_FREQ_5         0x2
#define MPU60X0_WAKE_FREQ_10        0x3

#define MPU60X0_BANKSEL_PRFTCH_EN_BIT       6
#define MPU60X0_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU60X0_BANKSEL_MEM_SEL_BIT         4
#define MPU60X0_BANKSEL_MEM_SEL_LENGTH      5

#define MPU60X0_WHO_AM_I_BIT        6
#define MPU60X0_WHO_AM_I_LENGTH     6

#define MPU60X0_DMP_MEMORY_BANKS        8
#define MPU60X0_DMP_MEMORY_BANK_SIZE    256
#define MPU60X0_DMP_MEMORY_CHUNK_SIZE   16                                          \

unsigned long int_time = 0;
unsigned long current_time = 0;
bool led_on = false;
bool interrupt_detected = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  // Wait for serial to initialize.
  while(!Serial) { }
  Serial.println("Test has begun");

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("LED pin mode set to output");
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED turned off");
  led_on = false;

  pinMode(INT_PIN, INPUT_PULLUP);
  Serial.println("INT_PIN set to pull-up");

  setISR();

  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up the MPU6050 (out of sleep mode)
  Wire.endTransmission();
  Serial.println("mpu initialized");

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // MOT_DETECT_CTRL register
  Wire.write(0x20); // Enable motion detection by setting ACCEL_INTEL_EN and MOT_EN bits
  Wire.endTransmission();
  Serial.println("mmpu motion detection init");

  mpu_set_int();

  readByte(MPU6050_ADDR, 58); // Reading will reset the interrupt line?
  
  // Deep sleep mode for 30 seconds, the ESP8266 wakes up by itself when GPIO 16 (D0 in NodeMCU board) is connected to the RESET pin
  //Serial.println("I'm awake, but I'm going into deep sleep mode for 30 seconds");
  //ESP.deepSleep(30e6); 
  /*
  Serial.println("Going into the delay now");
  delay(30000);
  Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal");
  ESP.deepSleep(0); 
  */
}

void loop() {
  //Serial.println("This is the loop");
  //delay(5000);
  // Deep sleep mode until RESET pin is connected to a LOW signal (for example pushbutton or magnetic reed switch)
  /*
  Serial.println("Going into the delay now");
  delay(10000);
  Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal");
  ESP.deepSleep(0);
  */
  
  if (interrupt_detected){
    interrupt_detected = false;
    Serial.println("interrupt detected code");
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("led on");
    led_on = true;
    int_time = millis();
    setISR();
  } 
  current_time = millis();
  if (current_time - int_time > 7000){
    digitalWrite(LED_BUILTIN, HIGH);
    if (led_on) {
      Serial.println("led off");
      led_on = false;
    }
  }
   delay(500);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.begin();
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.write(data); // Put data in Tx buffer
  Wire.endTransmission(); // Send the Tx buffer
  //  Serial.println("mnnj");

}

//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
  data = Wire.read(); // Fill Rx buffer with result
  return data; // Return data read from slave register
}

void mpu_set_int (){
  Serial.println("mpu_set_int begins");
  writeByte(MPU6050_ADDR, 0x6B, 0x00);
  writeByte(MPU6050_ADDR, MPU6050_SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  writeByte(MPU6050_ADDR, MPU60X0_RA_I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte(MPU6050_ADDR, MPU60X0_RA_ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte(MPU6050_ADDR, MPU60X0_RA_MOT_THR, 8); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  writeByte(MPU6050_ADDR, MPU60X0_RA_MOT_DUR, 40); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte(MPU6050_ADDR, MPU60X0_RA_MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms.)   
  writeByte(MPU6050_ADDR, MPU60X0_RA_INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
  writeByte(MPU6050_ADDR, 0x37, 160); // now INT pin is active low

  Serial.println("mpu_set_int done");
}

void IRAM_ATTR wakeUpFromInterrupt() {
  // This function will be called when motion is detected by the MPU6050
  // You can add any code you want to execute upon motion detection here
  detachInterrupt(0);

  readByte(MPU6050_ADDR, 58); // Reading will reset the interrupt line

  Serial.println("interrupted");
  interrupt_detected = true;
}

void setISR() {
  // Configure the interrupt to wake up from deep sleep
  attachInterrupt(digitalPinToInterrupt(INT_PIN), wakeUpFromInterrupt, FALLING);
  Serial.println("ISR attached");
}
