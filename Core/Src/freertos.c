/* freertos.c */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "bno055_stm32.h"
#include "shared_data.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1;

__attribute__((used)) int _printf_float;
/* USER CODE END Variables */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

__weak void vApplicationIdleHook(void) {}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)"STACK OVERFLOW: ", 16, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)pcTaskName, strlen((char*)pcTaskName), 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
}

__weak void vApplicationMallocFailedHook(void) {}

/* GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

/* ── 상수 정의 ──────────────────────────────── */
#define LPF_ALPHA  0.02f
#define DEAD_BAND  0.3f
#define G_THRESH   0.2f
#define ZUPT_HOLD  10
#define DT 0.01f

/* ── 헬퍼 함수 ──────────────────────────────── */
static inline float rad(float d) { return d * 0.01745329252f; }

void bodyToENU(float ax, float ay, float az,
               float yaw, float roll, float pitch,
               float *ex, float *ey, float *ez)
{
	float cy = cosf(rad(yaw)), sy = sinf(rad(yaw));
	float cp = cosf(rad(pitch)), sp = sinf(rad(pitch));
	float cr = cosf(rad(roll)), sr = sinf(rad(roll));

	float gx = cp*cy*ax + (sr*sp*cy - cr*sy)*ay + (cr*sp*cy + sr*sy)*az;
	float gy = cp*sy*ax + (sr*sp*sy + cr*cy)*ay + (cr*sp*sy - sr*cy)*az;
	float gz = -sp*ax + sr*cp*ay + cr*cp*az;

    *ex = gy; *ey = gx; *ez = -gz;
}

/* ── Task 전방 선언 ─────────────────────────── */
void SensorTask   (void const *argument);
void AdcsTask     (void const *argument);
void MissionTask  (void const *argument);
void TelemetryTask(void const *argument);
void CommandTask  (void const *argument);

/* ── FreeRTOS 초기화 ────────────────────────── */
void MX_FREERTOS_Init(void)
{
    /* Mutex를 Task보다 먼저 생성 */
    g_attitude_mutex = xSemaphoreCreateMutex();
    configASSERT(g_attitude_mutex != NULL);

    osThreadDef(Sensor,    SensorTask,    osPriorityNormal,      0, 256);
    osThreadCreate(osThread(Sensor),    NULL);

    osThreadDef(ADCS,      AdcsTask,      osPriorityBelowNormal, 0, 256);
    osThreadCreate(osThread(ADCS),      NULL);

    osThreadDef(Mission,   MissionTask,   osPriorityBelowNormal, 0, 256);
    osThreadCreate(osThread(Mission),   NULL);

    osThreadDef(Telemetry, TelemetryTask, osPriorityLow,         0, 256);
    osThreadCreate(osThread(Telemetry), NULL);

    osThreadDef(Command,   CommandTask,   osPriorityLow,         0, 256);
    osThreadCreate(osThread(Command),   NULL);
}



/* ── Sensor Task ────────────────────────────── */
void SensorTask(void const *argument)
{
    bno055_assignI2C(&hi2c1);
    bno055_setup();
    bno055_setOperationModeNDOF();
    osDelay(1000);

    float biasX=0, biasY=0, biasZ=0;
    float sumx=0, sumy=0, sumz=0;
    int cnt=0;
    float ax_f=0, ay_f=0, az_f=0;
    float vx=0, vy=0, vz=0;
    float px=0, py=0, pz=0;
    float totalDist=0;

    // 2초간 루프 실행
    unsigned long t0 = HAL_GetTick();
    while(HAL_GetTick()-t0<2000){

    	bno055_vector_t accel = bno055_getVectorLinearAccel();
    	sumx+=accel.x;
    	sumy+=accel.y;
    	sumz+=accel.z;
    	cnt++;
    	osDelay(10); //10ms마다 데이터 읽음
    }
    //평균값 계산
    biasX=sumx/cnt;
    biasY=sumy/cnt;
    biasZ=sumz/cnt;


    for (;;) {
    	//매 루프마다 자세(Euler)와 선형가속도 읽고 순수 가속도만 추출
    	bno055_vector_t euler = bno055_getVectorEuler();
    	bno055_vector_t lin   = bno055_getVectorLinearAccel();
    	float roll  = euler.y;
    	float pitch = euler.z;
    	float yaw   = euler.x;
    	float ax = lin.x - biasX;
    	float ay = lin.y - biasY;
    	float az = lin.z - biasZ;

    	//Body -> ENU
    	float ex, ey, ez;
    	bodyToENU(ax, ay, az, yaw, roll, pitch, &ex, &ey, &ez);  // 주소 넘기기

    	// LPF
        ax_f = (1.0f - LPF_ALPHA) * ax_f + LPF_ALPHA * ex;
        ay_f = (1.0f - LPF_ALPHA) * ay_f + LPF_ALPHA * ey;
        az_f = (1.0f - LPF_ALPHA) * az_f + LPF_ALPHA * ez;

        //Dead Band
        if (fabs(ax_f) < DEAD_BAND) ax_f = 0;
        if (fabs(ay_f) < DEAD_BAND) ay_f = 0;
        if (fabs(az_f) < DEAD_BAND) az_f = 0;

        //zupt
        bno055_vector_t gyro = bno055_getVectorGyroscope();
        float gNorm = sqrtf(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z);
        if(gNorm<G_THRESH){
        	vx=0;vy=0;vz=0;
        }

        //적분
        vx += ax_f * DT;
        vy += ay_f * DT;
        vz += az_f * DT;

        px += vx * DT;
        py += vy * DT;
        pz += vz * DT;

        totalDist += sqrtf(vx*vx + vy*vy) * DT;

        //mutex
        if (xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_attitude.yaw   = euler.x;  // Heading
            g_attitude.roll  = euler.y;
            g_attitude.pitch = euler.z;
            g_attitude.vx = vx;  g_attitude.vy = vy;  g_attitude.vz = vz;
            g_attitude.px = px;  g_attitude.py = py;  g_attitude.pz = pz;
            g_attitude.totalDist = totalDist;
            g_attitude.zupt = (gNorm < G_THRESH);
            g_attitude.ax_f = ax_f;
            g_attitude.ay_f = ay_f;
            xSemaphoreGive(g_attitude_mutex);
        }

        osDelay(10); // 100Hz

    }
}

/* ── ADCS Task ─────────────────────────── */
void AdcsTask(void const *argument)
{
	float prevYaw = 0.0f;
	bool firstLoop = true;
    for (;;) {
        //mutex로 g_attitude 읽기
    	AttitudeData_t snap;
    	if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    		snap = g_attitude;
    		xSemaphoreGive(g_attitude_mutex);
    	}else{
    		osDelay(50);
    		continue;
    	}

    	float dYaw = fabsf(snap.yaw - prevYaw);
    	if(dYaw > 180.0f) dYaw = 360.0f - dYaw;
    	prevYaw = snap.yaw;

    	if(!firstLoop &&
    		(fabsf(snap.roll) > 45.0f || fabsf(snap.pitch)>45.0f || dYaw > 30.0f)){

    	    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	        g_attitude.state = SYS_FAULT;
    	        g_attitude.faultCount++;
    	        xSemaphoreGive(g_attitude_mutex);
    	    }

    		bno055_setup();               // 레지스터 초기화 (동작 모드 설정)
    		bno055_setOperationModeNDOF(); // NDOF 모드 재진입
    		osDelay(1000); // 센서 안정화 대기

    	    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	        if(g_attitude.faultCount >= 3)
    	            g_attitude.state = SYS_SAFE_MODE;
    	        else
    	            g_attitude.state = SYS_NORMAL;
    	        xSemaphoreGive(g_attitude_mutex);
    	    }
    	}
    	firstLoop = false;

        osDelay(20); // 50Hz,20ms
    }
}

/* ── Mission Task ──────────────────────── */
void MissionTask(void const *argument)
{
	float initialYaw = 0.0f;
	float prevDist = 0.0f;
    for (;;) {
    	//mutex로 g_attitude 읽기
    	AttitudeData_t snap;
    	if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    		snap = g_attitude;
    		xSemaphoreGive(g_attitude_mutex);
    	}else{
    		osDelay(50);
    		continue;
    	}

    	float totalDist = snap.totalDist * g_distanceGain;

    	if(snap.missionStep==0){

    	}
    	//홀수 -> 전진
    	else if(snap.missionStep==1||snap.missionStep==3||snap.missionStep==5){

    		//한번 늘어난 거리는 줄어들지 않게
    		if(totalDist<prevDist) totalDist = prevDist;
    		else prevDist = totalDist;

    		//10m 전진 -> 다음스텝
    		if(totalDist >= 10.0f){
    			prevDist = 0.0f;
    			initialYaw = snap.yaw;
    		    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    		        g_attitude.missionStep++;
    		        xSemaphoreGive(g_attitude_mutex);
    		    }
    		}
    	}
    	//짝수 -> 회전
    	else if(snap.missionStep==2||snap.missionStep==4){

    		float yawChange = fabsf(snap.yaw -initialYaw);
    		if(yawChange > 180.0f) yawChange = 360.0f-yawChange;

    		if(yawChange >= 90.0f){
    			prevDist = 0.0f;
    	    	if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	    		g_attitude.totalDist = 0;
    	    		g_attitude.missionStep++;
    	    		xSemaphoreGive(g_attitude_mutex);
    	    	}
    		}
    	}
    	else if(snap.missionStep==6){
    	    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	        g_attitude.missionStep = 0;
    	        xSemaphoreGive(g_attitude_mutex);
    	    }
    	}

        osDelay(50); // 20Hz
    }
}

/* ── Telemetry Task ────────────────────── */
void TelemetryTask(void const *argument)
{
    for (;;) {
    	HAL_UART_Transmit(&huart1, (uint8_t*)"T\r\n", 3, 100);
        AttitudeData_t snap;

        //MUTEX
        if (xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            snap = g_attitude;  // 구조체 통째로 복사 후 즉시 반환
            xSemaphoreGive(g_attitude_mutex);
        }else{
        	osDelay(100);
        	continue;
        }

        char buf[128];
        int r = (int)snap.roll;
        int p = (int)snap.pitch;
        int y = (int)snap.yaw;
        int dist = (int)snap.totalDist;
        int st   = (int)snap.state;    // 0=NORMAL 1=FAULT 2=SAFE
        int ms   = snap.missionStep;
        int axf = (int)(snap.ax_f * 100);  // 소수점 2자리 표현
        int ayf = (int)(snap.ay_f * 100);
        int vx  = (int)(snap.vx  * 100);
        int vy  = (int)(snap.vy  * 100);
        int len = snprintf(buf, sizeof(buf),
            "R:%d P:%d Y:%d STEP:%d DIST:%d ST:%d AX:%d AY:%d VX:%d VY:%d\r\n",
			r, p, y, ms, dist, st, axf, ayf, vx, vy);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);

        osDelay(100); // 10Hz
    }
}

/* ── Command Task ──────────────────────── */
void CommandTask(void const *argument)
{
    uint8_t cmd;
    for (;;) {
        // 블로킹 수신 — 명령 올 때까지 여기서 기다림
        HAL_UART_Receive(&huart1, &cmd, 1, HAL_MAX_DELAY);

        if(cmd == 'S'){
            // Mutex로 missionStep = 1
    	    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	        g_attitude.missionStep = 1;
    	        xSemaphoreGive(g_attitude_mutex);
    	    }
        }
        else if(cmd == 'R'){
            // Mutex로 missionStep = 0, totalDist = 0, faultCount = 0
    	    if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	        g_attitude.missionStep = 0;
    	        g_attitude.totalDist = 0;
    	        g_attitude.faultCount = 0;
    	        xSemaphoreGive(g_attitude_mutex);
    	    }
        }
        else if(cmd == 'G'){
            // 숫자 문자열 추가 수신 후 g_distanceGain 업데이트
            char buf[8] = {0};
            HAL_UART_Receive(&huart1, (uint8_t*)buf, 7, 1000);
            g_distanceGain = (float)atof(buf);
        }
    }
}
