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
    HAL_UART_Transmit(&huart1, (uint8_t*)"BNO055 init OK\r\n", 16, 100);
    bno055_setOperationModeNDOF();
    osDelay(1000);

    for (;;) {
        bno055_vector_t v = bno055_getVectorEuler();

        if (xSemaphoreTake(g_attitude_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_attitude.yaw   = v.x;  // Heading
            g_attitude.roll  = v.y;
            g_attitude.pitch = v.z;
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

/* ── Mission Task 뼈대 ──────────────────────── */
void MissionTask(void const *argument)
{
    for (;;) {
        /* TODO Day 8: 전진/회전/완료 상태머신 */
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
