/*
 * bno055.h
 * Source: ivyknob/bno055_stm32 (BSD-3-Clause)
 * FreeRTOS support enabled via #define FREERTOS_ENABLED
 */

#ifndef BNO055_H_
#define BNO055_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ---------------------------------------------------------------
 * I2C Address
 * COM3 핀 GND → 0x28 (LO), VCC → 0x29 (HI)
 * 대부분의 모듈은 0x28
 * --------------------------------------------------------------- */
#define BNO055_I2C_ADDR_LO  0x28
#define BNO055_I2C_ADDR_HI  0x29
#define BNO055_I2C_ADDR     BNO055_I2C_ADDR_LO   // ← 필요시 HI로 변경

/* Chip ID */
#define BNO055_ID           0xA0

/* ---------------------------------------------------------------
 * Register Map (Page 0)
 * --------------------------------------------------------------- */
#define BNO055_CHIP_ID          0x00
#define BNO055_ACC_ID           0x01
#define BNO055_MAG_ID           0x02
#define BNO055_GYR_ID           0x03
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07

#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_EUL_DATA_X_LSB   0x1A
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_GRV_DATA_X_LSB   0x2E

#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42

#define BNO055_ACC_OFFSET_X_LSB 0x55

/* ---------------------------------------------------------------
 * Vector type defines
 * --------------------------------------------------------------- */
#define BNO055_VECTOR_ACCELEROMETER  BNO055_ACC_DATA_X_LSB
#define BNO055_VECTOR_MAGNETOMETER   BNO055_MAG_DATA_X_LSB
#define BNO055_VECTOR_GYROSCOPE      BNO055_GYR_DATA_X_LSB
#define BNO055_VECTOR_EULER          BNO055_EUL_DATA_X_LSB
#define BNO055_VECTOR_QUATERNION     BNO055_QUA_DATA_W_LSB
#define BNO055_VECTOR_LINEARACCEL    BNO055_LIA_DATA_X_LSB
#define BNO055_VECTOR_GRAVITY        BNO055_GRV_DATA_X_LSB

/* ---------------------------------------------------------------
 * Operation Modes
 * --------------------------------------------------------------- */
typedef enum {
  BNO055_OPERATION_MODE_CONFIG       = 0x00,
  BNO055_OPERATION_MODE_ACCONLY      = 0x01,
  BNO055_OPERATION_MODE_MAGONLY      = 0x02,
  BNO055_OPERATION_MODE_GYRONLY      = 0x03,
  BNO055_OPERATION_MODE_ACCMAG       = 0x04,
  BNO055_OPERATION_MODE_ACCGYRO      = 0x05,
  BNO055_OPERATION_MODE_MAGGYRO      = 0x06,
  BNO055_OPERATION_MODE_AMG          = 0x07,
  BNO055_OPERATION_MODE_IMU          = 0x08,
  BNO055_OPERATION_MODE_COMPASS      = 0x09,
  BNO055_OPERATION_MODE_M4G          = 0x0A,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
  BNO055_OPERATION_MODE_NDOF         = 0x0C,
} bno055_opmode_t;

/* ---------------------------------------------------------------
 * Axis Map
 * --------------------------------------------------------------- */
typedef enum {
  BNO055_AXIS_X = 0x00,
  BNO055_AXIS_Y = 0x01,
  BNO055_AXIS_Z = 0x02,
} bno055_axis_t;

typedef enum {
  BNO055_AXIS_SIGN_POSITIVE = 0x00,
  BNO055_AXIS_SIGN_NEGATIVE = 0x01,
} bno055_axis_sign_t;

typedef struct {
  bno055_axis_t      x;
  bno055_axis_sign_t x_sign;
  bno055_axis_t      y;
  bno055_axis_sign_t y_sign;
  bno055_axis_t      z;
  bno055_axis_sign_t z_sign;
} bno055_axis_map_t;

/* ---------------------------------------------------------------
 * Data Types
 * --------------------------------------------------------------- */
typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno055_vector_t;

typedef struct {
  uint8_t sys;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
} bno055_calibration_state_t;

typedef struct {
  struct { int16_t x, y, z; } accel;
  struct { int16_t x, y, z; } mag;
  struct { int16_t x, y, z; } gyro;
} bno055_offsets_t;

typedef struct {
  int16_t accel;
  int16_t mag;
} bno055_radius_t;

typedef struct {
  bno055_offsets_t offset;
  bno055_radius_t  radius;
} bno055_calibration_data_t;

typedef struct {
  uint8_t accState;
  uint8_t magState;
  uint8_t gyrState;
  uint8_t mcuState;
} bno055_self_test_result_t;

/* ---------------------------------------------------------------
 * Platform-specific functions (구현은 bno055_stm32.h 에 있음)
 * --------------------------------------------------------------- */

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */
void bno055_setup(void);
void bno055_setPage(uint8_t page);

bno055_opmode_t bno055_getOperationMode(void);
void bno055_setOperationMode(bno055_opmode_t mode);
void bno055_setOperationModeConfig(void);
void bno055_setOperationModeNDOF(void);

void bno055_enableExternalCrystal(void);
void bno055_disableExternalCrystal(void);
void bno055_reset(void);

int8_t   bno055_getTemp(void);
int16_t  bno055_getSWRevision(void);
uint8_t  bno055_getBootloaderRevision(void);
uint8_t  bno055_getSystemStatus(void);
uint8_t  bno055_getSystemError(void);

bno055_self_test_result_t   bno055_getSelfTestResult(void);
bno055_calibration_state_t  bno055_getCalibrationState(void);
bno055_calibration_data_t   bno055_getCalibrationData(void);
void bno055_setCalibrationData(bno055_calibration_data_t calData);

bno055_vector_t bno055_getVector(uint8_t vec);
bno055_vector_t bno055_getVectorAccelerometer(void);
bno055_vector_t bno055_getVectorMagnetometer(void);
bno055_vector_t bno055_getVectorGyroscope(void);
bno055_vector_t bno055_getVectorEuler(void);
bno055_vector_t bno055_getVectorLinearAccel(void);
bno055_vector_t bno055_getVectorGravity(void);
bno055_vector_t bno055_getVectorQuaternion(void);

void bno055_setAxisMap(bno055_axis_map_t axis);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H_ */
