/*
 * bno055.c
 * Source: ivyknob/bno055_stm32 (BSD-3-Clause)
 */

#include "bno055.h"
#include "bno055_stm32.h"
#include <string.h>
extern UART_HandleTypeDef huart1;
/* 단위 변환 스케일 */
static uint16_t accelScale       = 100;
static uint16_t tempScale        = 1;
static uint16_t angularRateScale = 16;
static uint16_t eulerScale       = 16;
static uint16_t magScale         = 16;
static uint16_t quaScale         = (1 << 14); /* 2^14 */

/* ---------------------------------------------------------------
 * 내부 유틸
 * --------------------------------------------------------------- */
void bno055_setPage(uint8_t page) {
  bno055_writeData(BNO055_PAGE_ID, page);
}

/* ---------------------------------------------------------------
 * Operation Mode
 * --------------------------------------------------------------- */
bno055_opmode_t bno055_getOperationMode(void) {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, (uint8_t *)&mode, 1);
  return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG)
    bno055_delay(19);
  else
    bno055_delay(7);
}

void bno055_setOperationModeConfig(void) {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF(void) {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

/* ---------------------------------------------------------------
 * 외부 크리스탈 / 리셋
 * --------------------------------------------------------------- */
static void bno055_setExternalCrystalUse(bool state) {
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x00;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

void bno055_enableExternalCrystal(void)  { bno055_setExternalCrystalUse(true);  }
void bno055_disableExternalCrystal(void) { bno055_setExternalCrystalUse(false); }

void bno055_reset(void) {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

/* ---------------------------------------------------------------
 * Setup (초기화)
 * --------------------------------------------------------------- */
void bno055_setup(void) {
  bno055_reset();
  HAL_Delay(100);

  uint8_t id = 0;
  for(int i=0; i<5; i++) {
	  bno055_readData(BNO055_CHIP_ID, &id, 1);
	  if(id == BNO055_ID) break;
	  HAL_Delay(50);
	}

  char buf[64];
  sprintf(buf, "Chip ID: 0x%02X\r\n", id);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);

  if (id != BNO055_ID) {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"BNO055 ID mismatch!\r\n", 21, 100);
  }

  bno055_setPage(0);
    bno055_writeData(BNO055_SYS_TRIGGER, 0x00);
    HAL_Delay(50);
    bno055_setOperationModeConfig();
    HAL_Delay(50);
}

/* ---------------------------------------------------------------
 * 시스템 정보
 * --------------------------------------------------------------- */
int8_t bno055_getTemp(void) {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return (int8_t)t;
}

int16_t bno055_getSWRevision(void) {
  bno055_setPage(0);
  uint8_t buf[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buf, 2);
  return (int16_t)((buf[1] << 8) | buf[0]);
}

uint8_t bno055_getBootloaderRevision(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemError(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

/* ---------------------------------------------------------------
 * Calibration
 * --------------------------------------------------------------- */
bno055_calibration_state_t bno055_getCalibrationState(void) {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys   = (calState >> 6) & 0x03;
  cal.gyro  = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag   =  calState       & 0x03;
  return cal;
}

bno055_calibration_data_t bno055_getCalibrationData(void) {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);
  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);
  memcpy(&calData.offset.accel, buffer,      6);
  memcpy(&calData.offset.mag,   buffer + 6,  6);
  memcpy(&calData.offset.gyro,  buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag,   buffer + 20, 2);
  bno055_setOperationMode(operationMode);
  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);
  memcpy(buffer,      &calData.offset.accel, 6);
  memcpy(buffer + 6,  &calData.offset.mag,   6);
  memcpy(buffer + 12, &calData.offset.gyro,  6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag,   2);
  for (uint8_t i = 0; i < 22; i++) {
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB + i, buffer[i]);
  }
  bno055_setOperationMode(operationMode);
}

/* ---------------------------------------------------------------
 * Vector 읽기 (핵심)
 * --------------------------------------------------------------- */
bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8] = {0};

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1.0;
  if      (vec == BNO055_VECTOR_MAGNETOMETER)                            scale = magScale;
  else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL   ||
           vec == BNO055_VECTOR_GRAVITY)                                 scale = accelScale;
  else if (vec == BNO055_VECTOR_GYROSCOPE)                               scale = angularRateScale;
  else if (vec == BNO055_VECTOR_EULER)                                   scale = eulerScale;
  else if (vec == BNO055_VECTOR_QUATERNION)                              scale = quaScale;

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};

  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer(void) { return bno055_getVector(BNO055_VECTOR_ACCELEROMETER); }
bno055_vector_t bno055_getVectorMagnetometer(void)  { return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);  }
bno055_vector_t bno055_getVectorGyroscope(void)     { return bno055_getVector(BNO055_VECTOR_GYROSCOPE);     }
bno055_vector_t bno055_getVectorEuler(void)         { return bno055_getVector(BNO055_VECTOR_EULER);         }
bno055_vector_t bno055_getVectorLinearAccel(void)   { return bno055_getVector(BNO055_VECTOR_LINEARACCEL);   }
bno055_vector_t bno055_getVectorGravity(void)       { return bno055_getVector(BNO055_VECTOR_GRAVITY);       }
bno055_vector_t bno055_getVectorQuaternion(void)    { return bno055_getVector(BNO055_VECTOR_QUATERNION);    }

/* ---------------------------------------------------------------
 * Axis Map
 * --------------------------------------------------------------- */
void bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap    = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign  = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN,   axisMapSign);
}
