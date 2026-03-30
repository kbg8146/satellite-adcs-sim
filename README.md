# 🛰️ GPS-Denied INS — STM32F429 + FreeRTOS

ESP32 + Arduino 환경에서 검증한 IMU 기반 INS 항법 알고리즘을  
**STM32F429 + STM32 HAL + FreeRTOS** 환경으로 이식하고,  
단일 루프 구조를 명시적 멀티태스크 구조로 재설계한 프로젝트입니다.

---

## 📌 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 보드 | STM32F429I-DISC1 |
| IMU | BNO055 (I2C1, PB6=SCL, PB7=SDA) |
| UART | USART1 (PA9=TX, 115200bps) |
| OS | FreeRTOS (CMSIS_V1) |
| IDE | STM32CubeIDE |

---

## 🔄 v1 vs v2 비교

| 구분 | v1 (ESP32 + Arduino) | v2 (STM32 + FreeRTOS) |
|------|----------------------|-----------------------|
| **플랫폼** | ESP32, Arduino Framework | STM32F429, STM32 HAL |
| **센서 드라이버** | `Adafruit_BNO055` 추상화 API | `HAL_I2C_Mem_Read/Write()` 직접 구현 |
| **구조** | `loop()` 단일 루프 순차 실행 | `xTaskCreate` 명시적 멀티태스크 (5개 Task) |
| **샘플링 타이밍** | TFT 렌더링 지연 시 100Hz 밀림 | Sensor Task 독립 실행, 100Hz 보장 |
| **명령 입력 대기** | `measuring = false`로 전체 정지 | Command Task만 대기, 나머지 계속 동작 |
| **데이터 공유** | 전역변수 무방비 접근 | Mutex 보호 공유 구조체 |
| **센서 오류 대응** | 없음 | Fault Recovery 상태머신 (NORMAL → FAULT → SAFE_MODE) |

---

## 🧵 시스템 아키텍처

### Task 구성

| Task | 주기 | 역할 |
|------|------|------|
| **Sensor Task** | 100Hz | BNO055 레지스터 직접 읽기, LPF, ZUPT, 적분 → 공유 구조체 업데이트 |
| **Mission Task** | 20Hz | 미션 상태머신 (전진/회전/완료 판정), Gain Factor 적용 |
| **ADCS Task** | 50Hz | 자세 이상 감지 (임계값 + 급격한 각속도 변화), Fault 판정 및 자동 복구 |
| **Command Task** | 이벤트 기반 | UART 명령 수신 (S/R/G), Gain Factor 런타임 입력 처리 |
| **Telemetry Task** | 10Hz | UART 상태 출력 (Roll, Pitch, Yaw, Step, Dist, State) |

### 데이터 흐름 (Mutex 보호)

```
Sensor Task ──(Mutex write)──→ g_attitude ──(Mutex read)──→ Mission Task
                                            ──(Mutex read)──→ ADCS Task
                                            ──(Mutex read)──→ Telemetry Task
```

- **읽기**: Mutex로 snap에 복사 → 이후 snap에서 읽기 (Mutex 불필요)
- **쓰기**: 반드시 Mutex로 감싸서 g_attitude에 직접 쓰기

---

## 🔬 알고리즘 파이프라인 (Sensor Task)

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

### 주요 파라미터

```c
#define LPF_ALPHA   0.02f   // Low Pass Filter 계수
#define DEAD_BAND   0.3f    // 가속도 Dead Band (m/s²)
#define G_THRESH    0.2f    // ZUPT 자이로 임계값 (rad/s)
#define DT          0.01f   // 샘플링 주기 (100Hz)
```

---

## 🔥 트러블슈팅

### 1. I2C Clock Stretching + SDA Bus Lock-up

**증상**: 주소 스캔(0x28)은 성공하지만, 데이터 읽기(0xA0)에서 `HAL_TIMEOUT` 무한 발생

**분석 과정**: ESP32 교차 검증으로 센서 하드웨어 정상 확인 → STM32 I2C 드라이버 문제로 범위 좁힘

**원인**: BNO055는 연산 중 SCL을 Low로 붙잡는 Clock Stretching 사용. STM32의 `NoStretchMode = ENABLE` 상태에서 이를 '라인 고장'으로 판단하여 통신 차단

**해결**:
- `NoStretchMode = DISABLE`로 SCL hold 허용
- 9-Clock Recovery로 SDA 버스 마비 해제
- NVIC 우선순위 상향 (FreeRTOS 스케줄링보다 I2C 인터럽트 우선 처리)

### 2. snprintf float 스택 오버플로우

**증상**: `òòòòòò` 깨진 문자 반복 출력, R/P/Y 값 비어있음

**원인**: `nano.specs` 환경에서 `snprintf("%.1f")`의 float printf가 내부적으로 300~500 bytes 스택 추가 사용 → TelemetryTask 스택 오버플로우

**해결**: float 출력을 정수 캐스팅으로 교체: `int r = (int)snap.roll;` + `snprintf("%d")`

**교훈**: ARM Cortex-M4에서 `%f`는 변수 크기(4B)와 무관하게 IEEE 754 비트 분리 + 소수점 반올림 처리의 로컬 변수가 스택에 수백 bytes 쌓임

### 3. 알고리즘 이식 순서 오류

**증상**: LPF가 Body 좌표계 데이터에 적용되어 좌표 변환 후 필터링 효과 왜곡

**원인**: LPF를 bodyToENU 변환 전에 배치하는 코드 순서 실수

**해결**: 올바른 순서 확립: `bodyToENU → LPF → Dead Band → ZUPT → 적분 → Mutex write`

### 4. MissionTask 지역변수 → 공유 구조체 재설계

**증상**: Command Task에서 미션 시작/리셋이 불가능

**원인**: `step`이 MissionTask 지역변수여서 다른 Task에서 접근 불가

**해결**: `shared_data.h`에 `missionStep` 필드 추가, Mutex를 통해 Task 간 공유. 읽기는 snap, 쓰기는 Mutex 안에서 g_attitude 직접 수정

### 5. 부팅 시 false FAULT 발생

**증상**: 시스템 시작 직후 ADCS가 즉시 FAULT 판정

**원인**: BNO055 초기 데이터가 불안정한 상태에서 Fault 조건 체크

**해결**: `firstLoop` 플래그로 최초 N사이클 Fault 판정 skip

---

## 🛡️ Fault Recovery 상태머신

```
           센서 오류 감지 (|Roll|>45° or |Pitch|>45° or dYaw>4°/frame)
NORMAL ─────────────────────────────────────────────→ FAULT
  ↑                                                    │
  │   재초기화 성공 (BNO055 재설정 + 바이어스 재측정)   │ 최대 3회 시도
  └────────────────────────────────────────────────────┘
                                               │ 3회 모두 실패
                                               ↓
                                          SAFE_MODE
                                  (Mission 중단, UART 경고 출력만 유지)
```

---

## 🗺️ 미션 시퀀스 (L자 경로)

```
STEP 0 → 대기 (UART 'S' 명령 대기)
STEP 1 → 전진 10m
STEP 2 → 우회전 90°
STEP 3 → 전진 10m
STEP 4 → 우회전 90°
STEP 5 → 전진 10m
STEP 6 → 완료 → STEP 0으로 리셋
```

---

## 📡 UART 인터페이스

### 명령어

| 명령 | 동작 |
|------|------|
| `S` | 미션 시작 (missionStep = 1) |
| `R` | 전체 리셋 (step=0, dist=0, fault=0) |
| `G1.5` | Distance Gain Factor = 1.5 설정 |

### Telemetry 출력 (10Hz)

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

## 📁 파일 구조

```
Core/
├── Inc/
│   ├── shared_data.h       # 공유 구조체 (AttitudeData_t) + Mutex extern 선언
│   └── bno055_stm32.h      # BNO055 HAL 드라이버
└── Src/
    ├── shared_data.c       # 전역 변수 메모리 할당
    ├── freertos.c          # 5개 FreeRTOS Task 구현
    └── main.c              # 주변장치 초기화, I2C Bus Recovery
```

---

## 🧩 관련 프로젝트

- **v1 (ESP32 + Arduino)**: 단일 루프 기반 INS 검증 버전 — ㄷ자 경로 2m 이내 오차 달성
