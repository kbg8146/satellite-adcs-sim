# 🛰️ Drone INS v2 — STM32F429 + FreeRTOS

ESP32 + Arduino 환경에서 검증한 IMU 기반 INS 항법 알고리즘을  
STM32F429 + STM32 HAL + FreeRTOS 환경으로 이식한 프로젝트입니다.

---

## 📌 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 보드 | STM32F429I-DISC1 |
| IMU | BNO055 (I2C1, PB6=SCL, PB7=SDA) |
| UART | USART1 (PA9=TX, 115200bps) |
| OS | FreeRTOS CMSIS_V1 |
| IDE | STM32CubeIDE |

---

## 🔄 v1 vs v2 비교

| 항목 | v1 (ESP32 + Arduino) | v2 (STM32 + FreeRTOS) |
|------|----------------------|-----------------------|
| 구조 | 단일 루프 (`loop()`) | 5개 독립 FreeRTOS Task |
| 제어 | 없음 | UART 명령어 (S/R/G) |
| 상태 모니터링 | `Serial.print` | Telemetry Task 10Hz |
| Fault Recovery | 없음 | ADCS Task 자동 복구 |
| 동기화 | 없음 | Mutex 기반 공유 데이터 |

---

## 🧵 Task 구성

```
┌─────────────────┬──────────┬──────────────────────────────────────┐
│ Task            │ 주기     │ 역할                                 │
├─────────────────┼──────────┼──────────────────────────────────────┤
│ SensorTask      │ 100Hz    │ BNO055 읽기, LPF, ZUPT, 적분        │
│ AdcsTask        │ 50Hz     │ Fault 감지 및 자동 복구             │
│ MissionTask     │ 20Hz     │ 전진/회전 상태머신 (L자 경로)       │
│ TelemetryTask   │ 10Hz     │ UART 상태 출력                      │
│ CommandTask     │ 이벤트   │ UART 명령 수신 (S/R/G)             │
└─────────────────┴──────────┴──────────────────────────────────────┘
```

---

## 📡 UART 명령어

| 명령 | 동작 |
|------|------|
| `S` | 미션 시작 (missionStep = 1) |
| `R` | 전체 리셋 (step=0, dist=0, fault=0) |
| `G1.5` | Distance Gain Factor = 1.5 설정 |

---

## 📊 Telemetry 출력 형식

```
R:{roll} P:{pitch} Y:{yaw} STEP:{step} DIST:{dist} ST:{state}
```

| 필드 | 설명 |
|------|------|
| R/P/Y | Roll, Pitch, Yaw (도, 정수) |
| STEP | 미션 단계 (0=대기, 1~5=진행, 6=완료) |
| DIST | 누적 이동 거리 (m, 정수) |
| ST | 시스템 상태 (0=NORMAL, 1=FAULT, 2=SAFE_MODE) |

---

## 🗺️ 미션 시퀀스 (L자 경로)

```
STEP 0 → 대기 (S 명령 대기)
STEP 1 → 전진 10m
STEP 2 → 우회전 90°
STEP 3 → 전진 10m
STEP 4 → 우회전 90°
STEP 5 → 전진 10m
STEP 6 → 완료 → STEP 0으로 리셋
```

---

## 🔧 알고리즘 파이프라인 (SensorTask)

```
BNO055 읽기
    │
    ├── Euler (Roll/Pitch/Yaw)
    └── LinearAccel (ax, ay, az)
         │
         ├─ 바이어스 보정 (부팅 시 2초 평균)
         ├─ Body → ENU 좌표 변환 (회전행렬)
         ├─ LPF (α=0.02)
         ├─ Dead Band 제거 (0.3 m/s²)
         ├─ ZUPT (자이로 norm < 0.2 rad/s → v=0)
         └─ 적분 → 속도 → 위치 → 누적 거리
```

---

## ⚙️ 주요 파라미터

```c
#define LPF_ALPHA   0.02f   // Low Pass Filter 계수
#define DEAD_BAND   0.3f    // 가속도 Dead Band (m/s²)
#define G_THRESH    0.2f    // ZUPT 자이로 임계값 (rad/s)
#define DT          0.01f   // 샘플링 주기 (100Hz)
```

---

## 🛡️ Fault Recovery (AdcsTask)

FAULT 조건: `|roll| > 45°` or `|pitch| > 45°` or `dYaw > 30°/cycle`

```
FAULT 감지
    → SYS_FAULT 상태 기록
    → faultCount++
    → BNO055 재초기화 (NDOF 모드)
    → 1초 대기
    → faultCount >= 3 ? SYS_SAFE_MODE : SYS_NORMAL
```

---

## 🔒 동기화 설계

모든 Task는 `g_attitude` 구조체를 공유합니다.

```
읽기: Mutex로 snap에 복사 → 이후 snap에서 읽기 (Mutex 불필요)
쓰기: 반드시 Mutex로 감싸서 g_attitude에 직접 쓰기
```

---

## 📁 파일 구조

```
Core/
├── Inc/
│   ├── shared_data.h       # 공유 구조체 및 전역 변수 선언
│   └── bno055_stm32.h      # BNO055 HAL 드라이버
└── Src/
    ├── shared_data.c       # 전역 변수 정의
    ├── freertos.c          # 모든 FreeRTOS Task 구현
    └── main.c              # 주변장치 초기화, I2C Bus Recovery
```

---

## 🧩 관련 프로젝트

- **v1**: [ESP32 + Arduino INS](https://github.com/kbg8146) — 단일 루프 기반 검증 버전
