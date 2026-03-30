#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include "BluetoothSerial.h"

// ===== TFT 추가 (ST7789) =====
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#define TFT_CS   5
#define TFT_DC   17
#define TFT_RST  16
// ESP32 핀 구성에 맞게 CS, DC, RST 핀 번호 확인 및 수정 필요
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ===== 객체 생성 =====
Adafruit_BNO055 bno(55, 0x28);
BluetoothSerial SerialBT;

// ===== 설정 =====
const unsigned long LOOP_INTERVAL_US = 10000; // 100Hz 샘플링
const unsigned long PRINT_INTERVAL_MS = 100;  // 10Hz 출력
unsigned long last_us = 0;
unsigned long lastPrintMs = 0;

// 미션 관련 상태 변수
bool measuring = false;
unsigned long start_ms = 0;
// ⚡️ Gain Factor 입력 대기 상태 추가
bool waitingForGain = false; 

// ==== LPF 설정 ====
const float LPF_ALPHA = 0.02f;
float ax_f = 0, ay_f = 0, az_f = 0;

// ==== ZUPT 설정 ====
const float G_THRESH  = 0.2f;     
const float DEAD_BAND = 0.3f;
const int   ZUPT_HOLD = 10;
int   zuptCount = 0;

// ==== 바이어스 ====
float biasX = 0, biasY = 0, biasZ = 0;

// 순수 적분 상태
float px = 0, py = 0, pz = 0;
float vx = 0, vy = 0, vz = 0;

// 미션 관련 변수
int missionStep = 0;
float initialYaw = 0.0f;
float currentYaw = 0.0f;
float totalDistRaw = 0.0f; // raw total distance
float prevDistXY = 0.0f;

// ⚡️ 거리 보정 Gain Factor (변수가 되어 동적으로 변경 가능)
float DISTANCE_GAIN = 1.0f; // 초기값 1.0f (보정 없음)

// 미션 목표값
const float TARGET_DIST_10M = 10.0f;
const float TARGET_YAW_CHANGE = 90.0f;

// ===== Body → ENU 변환 =====
inline float rad(float d) { return d * 0.01745329252f; }

void bodyToENU(float ax, float ay, float az, float yaw, float roll, float pitch,
               float &ex, float &ey, float &ez) {
  float cy = cosf(rad(yaw)), sy = sinf(rad(yaw));
  float cp = cosf(rad(pitch)), sp = sinf(rad(pitch));
  float cr = cosf(rad(roll)), sr = sinf(rad(roll));

  float gx = cp * cy * ax + (sr * sp * cy - cr * sy) * ay + (cr * sp * cy + sr * sy) * az;
  float gy = cp * sy * ax + (sr * sp * sy + cr * cy) * ay + (cr * sp * sy - sr * cy) * az;
  float gz = -sp * ax + sr * cp * ay + cr * cp * az;

  ex = gy; ey = gx; ez = -gz;
}

void dualPrintf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  SerialBT.print(buf);
}

// ===== TFT 함수 =====
void setupTFT(){
  tft.init(240, 320); 
  tft.setRotation(1); 
  
  // 🔥 Mirror X (좌우반전) - 사용하는 모듈에 따라 필요할 수 있음
  tft.startWrite();
  tft.writeCommand(0x36);
  tft.spiWrite(0b00101000);
  tft.endWrite();

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
}

void drawToTFT(float elapsed, int step, float ax, float ay, float totalDist, float yawChange, bool zupt) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);

  tft.setTextColor(ST77XX_WHITE);
  tft.printf("Step: %d (t:%.1fs)\n", step, elapsed);
  tft.drawFastHLine(0, 20, tft.width(), ST77XX_BLUE);

  // 전진 미션 (1, 3, 5)
  if (step == 1 || step == 3 || step == 5) {
      tft.setTextColor(ST77XX_CYAN);
      tft.printf("DIST: %.3f / %.1f m\n", totalDist, TARGET_DIST_10M); 
      tft.setTextColor(ST77XX_YELLOW);
      tft.printf("ax: %.2f ay: %.2f\n", ax, ay);
      tft.setTextColor(zupt ? ST77XX_RED : ST77XX_GREEN);
      tft.printf("ZUPT: %s\n", zupt ? "YES" : "NO");
      tft.setTextColor(ST77XX_WHITE);
      tft.println("MOVE...");
  } 
  // 회전 미션 (2, 4)
  else if (step == 2 || step == 4) {
      tft.setTextColor(ST77XX_MAGENTA);
      tft.printf("YAW CHG: %.2f / %.1f deg\n", yawChange, TARGET_YAW_CHANGE);
      tft.setTextColor(ST77XX_YELLOW);
      tft.printf("Current YAW: %.2f\n", currentYaw);
      tft.setTextColor(ST77XX_WHITE);
      tft.println("ROTATING...");
  }
}

// ===== 바이어스 재측정 함수 추가 =====
void recalibrateBias() {
  dualPrintf("⏳ 2초간 정지하여 바이어스 재측정 중...\n");
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_ORANGE);
  tft.println("Re-Calibrating Bias...");

  float sumx = 0, sumy = 0, sumz = 0; 
  int cnt = 0; 
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    sensors_event_t eLin;
    bno.getEvent(&eLin, Adafruit_BNO055::VECTOR_LINEARACCEL);
    sumx += eLin.acceleration.x;
    sumy += eLin.acceleration.y;
    sumz += eLin.acceleration.z;
    cnt++;
    delay(10);
  }
  biasX = sumx / cnt; 
  biasY = sumy / cnt; 
  biasZ = sumz / cnt;
  dualPrintf("✅ Bias Recalibrated: (%.3f, %.3f, %.3f)\n", biasX, biasY, biasZ);
}

// ===== 미션 변수 초기화 =====
void resetMission() {
  px = py = pz = 0;
  vx = vy = vz = 0;
  ax_f = ay_f = az_f = 0;
  zuptCount = 0;
  totalDistRaw = 0.0f; 
  prevDistXY = 0.0f;
}

// ===== 미션 시작 =====
void startMission(int step) {
  resetMission();
  missionStep = step;
  start_ms = millis();
  measuring = true;

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(3);

  if (step == 1 || step == 3 || step == 5) {
    // ⚡️ 전진 미션: Gain 입력 대기로 전환
    waitingForGain = true;
    measuring = false; // Gain 입력 중에는 측정 정지
    
    tft.printf("MISSION %d\n", step);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_ORANGE);
    tft.println("Input Gain Factor:");
    tft.printf("(Current: %.3f)", DISTANCE_GAIN);
    dualPrintf("\n▶️ Mission %d: Gain Factor를 입력하세요. (예: 1.4)\n", step);
    dualPrintf("현재 Gain: %.3f. 입력 후 Enter를 누르세요.\n", DISTANCE_GAIN);
    
  } else if (step == 2 || step == 4) {
    // 회전 미션: 즉시 시작
    sensors_event_t eOri; 
    bno.getEvent(&eOri);
    initialYaw = eOri.orientation.x;
    tft.printf("ROTATE %d\n", step);
    tft.setTextSize(2);
    dualPrintf("▶️ Mission %d: %.1f 도 회전 시작 (시작 Yaw: %.2f)\n", step, TARGET_YAW_CHANGE, initialYaw);
  } else if (step == 6) {
    tft.printf("MISSION\nCOMPLETE");
    dualPrintf("✅ Mission 6: 모든 미션 완료! 최종 ㄷ자 경로 완성。\n");
    measuring = false;
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_IMU_MISSION");

  // ===== TFT 초기화 =====
  setupTFT();
  
  dualPrintf("Bluetooth Setup...\n");

  if (!bno.begin()) {
    dualPrintf("❌ BNO055 not found\n");
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(0,0);
    tft.println("BNO055 ERROR!");
    while (1);
  }

  bno.setMode(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);

  dualPrintf("⏳ 2초간 정지하여 초기 바이어스 측정...\n");
  tft.setCursor(0,0);
  tft.println("Initial Calib...");

  float sumx = 0, sumy = 0, sumz = 0; 
  int cnt = 0; 
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    sensors_event_t eLin; 
    bno.getEvent(&eLin, Adafruit_BNO055::VECTOR_LINEARACCEL);
    sumx += eLin.acceleration.x;
    sumy += eLin.acceleration.y;
    sumz += eLin.acceleration.z;
    cnt++; delay(10);
  }
  biasX = sumx / cnt; 
  biasY = sumy / cnt; 
  biasZ = sumz / cnt;

  dualPrintf("✅ Initial Bias: (%.3f, %.3f, %.3f)\n", biasX, biasY, biasZ);
  dualPrintf("✅ 's' 입력 시 Mission 1 시작\n"); 
  
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
  tft.printf("Bias OK!\n(%.3f,%.3f,%.3f)\n", biasX, biasY, biasZ);
  tft.println("Press 's' to Start");

  last_us = micros();
  missionStep = 0;
}

void handleInput() {
  // ⚡️ Gain Factor 입력 대기 로직
  if (waitingForGain) {
    // 다중 문자(float) 입력을 처리하기 위한 버퍼
    static String gainInput = ""; 
    
    // 시리얼/블루투스 입력 버퍼의 모든 데이터를 읽음
    while (Serial.available() || SerialBT.available()) {
      char c = 0;
      if (Serial.available()) c = Serial.read();
      else if (SerialBT.available()) c = SerialBT.read();

      if (c == '\n' || c == '\r') { // 입력 종료 (Enter/Newline)
        if (gainInput.length() > 0) {
          float newGain = gainInput.toFloat();
          // 단순 유효성 검사 (0.1 ~ 10.0 사이의 값만 허용)
          if (newGain > 0.1f && newGain < 10.0f) { 
            DISTANCE_GAIN = newGain;
            dualPrintf("\n✅ Gain Factor %.3f 로 설정 완료.\n", DISTANCE_GAIN);
            waitingForGain = false;
            measuring = true; // 측정 시작 허용

            // Gain 설정 후, 원래 startMission에서 하려던 미션 준비 단계를 실행
            recalibrateBias(); 
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(0, 0);
            tft.setTextSize(3);
            tft.printf("MISSION %d\n", missionStep);
            tft.setTextSize(2);
            dualPrintf("▶️ Mission %d (Gain %.3f) 전진 시작\n", missionStep, DISTANCE_GAIN);
            
            start_ms = millis(); // 타이머 재시작 (측정 시작)
          } else {
            dualPrintf("\n❌ 유효하지 않은 Gain Factor입니다. 0.1 ~ 10.0 사이의 값을 입력하세요.\n");
          }
        }
        gainInput = ""; // 입력 버퍼 초기화
        break; // 입력 종료
      } else if ((c >= '0' && c <= '9') || c == '.') {
        // 숫자나 소수점 입력 받기
        gainInput += c;
        dualPrintf("%c", c); // 입력 값 에코
      }
    }
    return; // Gain 입력 대기 중에는 다른 명령 처리 안함
  }

  // 일반 명령 처리 ('s'로 Mission 1 시작)
  char c = 0;
  if (Serial.available()) c = Serial.read();
  else if (SerialBT.available()) c = SerialBT.read();
  if (!c) return;

  if (c == 's' || c == 'S') {
    if (missionStep == 0 || !measuring) { // 미션이 시작되지 않았거나 완료된 상태
        startMission(1);
    }
  }
}

float normalizeYaw(float yaw) {
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  return yaw;
}

void loop() {
  handleInput();

  unsigned long now = micros();
  if (now - last_us < LOOP_INTERVAL_US) return;
  float dt = (now - last_us) * 1e-6f;
  last_us = now;

  // Gain 입력 대기 중이거나 미션이 완료된 경우 측정 중지
  if (!measuring) return; 

  sensors_event_t eLin, eOri, eGyro;
  bno.getEvent(&eLin, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&eOri);
  bno.getEvent(&eGyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

  float roll = eOri.orientation.y;
  float pitch = eOri.orientation.z;
  currentYaw = eOri.orientation.x;

  float ax, ay, az;
  bodyToENU(eLin.acceleration.x - biasX,
            eLin.acceleration.y - biasY,
            eLin.acceleration.z - biasZ,
            currentYaw, roll, pitch, ax, ay, az);

  ax_f = (1 - LPF_ALPHA) * ax_f + LPF_ALPHA * ax;
  ay_f = (1 - LPF_ALPHA) * ay_f + LPF_ALPHA * ay;
  az_f = (1 - LPF_ALPHA) * az_f + LPF_ALPHA * az;
  ax = ax_f; ay = ay_f; az = az_f;

  if (fabs(ax) < DEAD_BAND) ax = 0;
  if (fabs(ay) < DEAD_BAND) ay = 0;
  if (fabs(az) < DEAD_BAND) az = 0;

  bool isAccelZero = (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f);
  float gmag = sqrtf(eGyro.gyro.x * eGyro.gyro.x + eGyro.gyro.y * eGyro.gyro.y + eGyro.gyro.z * eGyro.gyro.z);
  bool zuptNow = isAccelZero && (gmag < G_THRESH);
  if (zuptNow) zuptCount++; else zuptCount = 0;
  bool zupt = (zuptCount >= ZUPT_HOLD);

  if (zupt) { vx = vy = vz = 0.0f; }
  else {
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;
    // 2D 평면 이동 거리를 deltaDist로 계산하여 totalDistRaw에 누적
    float deltaDist = hypotf(vx * dt, vy * dt); 
    totalDistRaw += deltaDist; // raw distance에 누적
    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;
  }

  // ✅ Gain Factor 적용하여 최종 totalDist 계산
  float totalDist = totalDistRaw * DISTANCE_GAIN;

  float distXY = hypotf(px, py);

  // ===== 미션 단계 완료 체크 =====
  if (missionStep == 1 || missionStep == 3 || missionStep == 5) {
    float targetDist = TARGET_DIST_10M;
    if (distXY < prevDistXY) distXY = prevDistXY;
    else prevDistXY = distXY;

    if (totalDist >= targetDist) { // Gain이 적용된 totalDist를 기준으로 체크
      dualPrintf("🎉 Mission %d 완료! (이동 거리: %.3f m)\n", missionStep, totalDist);
      startMission(missionStep + 1);
      return;
    }

  } else if (missionStep == 2 || missionStep == 4) {
    float yawChange = fabs(normalizeYaw(currentYaw - initialYaw));
    if (yawChange >= TARGET_YAW_CHANGE) {
      dualPrintf("🎉 Mission %d 완료! (Yaw 변화: %.2f 도)\n", missionStep, yawChange);
      startMission(missionStep + 1);
      return;
    }
  }

  // ===== 10Hz 출력 / TFT 업데이트 =====
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = nowMs;

  float elapsed = (nowMs - start_ms) / 1000.0f;
  float yawChange = (missionStep == 2 || missionStep == 4) ? fabs(normalizeYaw(currentYaw - initialYaw)) : 0.0f;

  dualPrintf("Step:%d t:%5.2fs a:(%6.3f %6.3f %6.3f) v:(%6.3f %6.3f %6.3f) RPY:(%6.2f %6.2f %6.2f) Dist:%7.3f m YawChange:%5.2f ZUPT:%c gMag:%6.3f\n",
             missionStep, elapsed, ax, ay, az, vx, vy, vz, roll, pitch, currentYaw,
             totalDist, yawChange, (zupt ? 'Y' : 'N'), gmag);

  // ===== TFT 출력 함수 호출 =====
  drawToTFT(elapsed, missionStep, ax, ay, totalDist, yawChange, zupt);
}
