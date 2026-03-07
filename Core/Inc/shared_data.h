#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>

/* ── 시스템 상태 열거형 ─────────────────────── */
typedef enum {
    SYS_NORMAL,
    SYS_FAULT,
    SYS_SAFE_MODE
} SystemState_t;

/* ── 공유 데이터 구조체 ─────────────────────── */
typedef struct {
    /* 자세 (BNO055 Euler, deg) */
    float roll;
    float pitch;
    float yaw;
    /* 속도 (m/s) */
	float vx, vy, vz;

	/* 위치 (m) */
	float px, py, pz;

	/* 누적 이동 거리 (m) */
	float totalDist;

    /* ZUPT 상태 */
    bool zupt;

    /* 시스템 상태 */
    SystemState_t state;

    /* Fault Recovery 재시도 횟수 */
    uint8_t faultCount;
} AttitudeData_t;

/* ── 외부 공개 변수 ─────────────────────────── */
extern AttitudeData_t    g_attitude;
extern SemaphoreHandle_t g_attitude_mutex;

#endif /* SHARED_DATA_H */
