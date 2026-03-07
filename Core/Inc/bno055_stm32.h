/*
 * bno055_stm32.h
 * STM32 HAL I2C 플랫폼 레이어
 * Source: ivyknob/bno055_stm32 (BSD-3-Clause)
 *
 * [사용법]
 * 1. main.c USER CODE BEGIN Includes에 추가:
 *      #include "bno055_stm32.h"
 * 2. USER CODE BEGIN 2에 추가:
 *      bno055_assignI2C(&hi2c1);
 *      bno055_setup();
 *      bno055_setOperationModeNDOF();
 *
 * [FreeRTOS 사용 시]
 *   아래 주석 해제:
 *   #define FREERTOS_ENABLED
 */

#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>


// 아래 두 줄을 추가하세요
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1; // main.c에 있는 huart1을 참조하기 위해 추가
/* FreeRTOS 사용 시 주석 해제 */
#define FREERTOS_ENABLED

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

#include "bno055.h"

/* I2C 핸들 포인터 (전역) */
static I2C_HandleTypeDef *_bno055_i2c_port;

/* I2C 핸들 등록 */
static inline void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _bno055_i2c_port = hi2c_device;
}

/* Delay 함수: FreeRTOS 사용 시 osDelay, 아니면 HAL_Delay */
static inline void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

static inline void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  // HAL_I2C_Mem_Read 사용, 타임아웃 1000ms
  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x28 << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000) != HAL_OK) {
    // 에러 발생 시 로그 (USART1 가정)
    HAL_UART_Transmit(&huart1, (uint8_t*)"BNO Read Error!\r\n", 17, 100);
  }
}

static inline void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata = data;
  // HAL_I2C_Mem_Write 사용, 타임아웃 1000ms
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x28 << 1), reg, I2C_MEMADD_SIZE_8BIT, &txdata, 1, 1000) != HAL_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"BNO Write Error!\r\n", 18, 100);
  }
}

#ifdef __cplusplus
}
#endif

#endif /* BNO055_STM32_H_ */
