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
#include <stdio.h>
#include <math.h>
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
            xSemaphoreGive(g_attitude_mutex);
        }

        osDelay(10); // 100Hz

    }
}

/* ── ADCS Task 뼈대 ─────────────────────────── */
void AdcsTask(void const *argument)
{
    for (;;) {
        /* TODO Day 9: 임계값 초과 / 급격한 각속도 변화 감지 */
        osDelay(20); // 50Hz
    }
}

/* ── Mission Task ──────────────────────── */
void MissionTask(void const *argument)
{
	int step = 0;
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

    	if(step==0){

    	}
    	//홀수 -> 전진
    	else if(step==1||step==3||step==5){

    		//한번 늘어난 거리는 줄어들지 않게
    		if(totalDist<prevDist) totalDist = prevDist;
    		else prevDist = totalDist;

    		//10m 전진 -> 다음스텝
    		if(totalDist >= 10.0f){
    			prevDist = 0.0f;
    			initialYaw = snap.yaw;
    			step++;
    		}
    	}
    	//짝수 -> 회전
    	else if(step==2||step==4){

    		float yawChange = fabsf(snap.yaw -initialYaw);
    		if(yawChange > 180.0f) yawChange = 360.0f-yawChange;

    		if(yawChange >= 90.0f){
    			prevDist = 0.0f;
    	    	if(xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(10))==pdTRUE){
    	    		g_attitude.totalDist = 0;
    	    		xSemaphoreGive(g_attitude_mutex);
    	    	}
    			step++;
    		}
    	}
    	else if(step==6){
    		step = 0;
    	}

        osDelay(50); // 20Hz
    }
}

/* ── Telemetry Task 뼈대 ────────────────────── */
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
        int len = snprintf(buf, sizeof(buf),
            "R:%d P:%d Y:%d\r\n", r, p, y);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);

        osDelay(100); // 10Hz
    }
}

/* ── Command Task 뼈대 ──────────────────────── */
void CommandTask(void const *argument)
{
    for (;;) {
        /* TODO Day 10: UART 명령 수신 + Gain Factor 런타임 조정 */
        osDelay(100);
    }
}
