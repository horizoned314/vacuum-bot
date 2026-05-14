// ============================================================================
// VacBot SLAM Navigator — ESP32 Main Firmware  (Batch 2)
// ----------------------------------------------------------------------------
// Target  : ESP32 DevKit V1 (Arduino-ESP32 core 2.0.14+ or 3.x)
// Role    : Real-time sensing, odometry+IMU fusion, motor control,
//           UDP telemetry/command, OLED status, buzzer alerts.
//
// LOCKED UDP CONTRACT (do not change):
//   TX (ESP -> PC) : {"x":f,"y":f,"theta":f,"distances":[F,L,R],"timestamp":u}
//   RX (PC -> ESP) : {"left_speed":i,"right_speed":i}
//
// Units  : meters, radians.  Angles wrapped to [-pi, pi].
// Frame  : +x forward, +y left, +theta CCW (consistent w/ PC side).
//
// Library deps (install via Arduino Library Manager):
//   - ArduinoJson  v6.x      (we use StaticJsonDocument — RX only)
//   - Adafruit GFX
//   - Adafruit SSD1306
// Built-ins used: Wire, WiFi, WiFiUdp.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ───────────── User configuration ─────────────────────────────────────────
static const char*    WIFI_SSID = "YOUR_SSID";
static const char*    WIFI_PASS = "YOUR_PASSWORD";
static const IPAddress PC_IP(192, 168, 1, 100);   // PC listens here
static const uint16_t  PC_PORT  = 9000;            // ESP32 -> PC telemetry
static const uint16_t  ESP_PORT = 9001;            // PC -> ESP32 commands

// ───────────── Pin map (locked by hardware spec) ──────────────────────────
// I2C bus (MPU6050 + OLED share it)
static const uint8_t I2C_SDA   = 21;
static const uint8_t I2C_SCL   = 22;

// HC-SR04 — front, left, right
static const uint8_t US_F_TRIG = 19, US_F_ECHO = 18;
static const uint8_t US_L_TRIG = 25, US_L_ECHO = 26;
static const uint8_t US_R_TRIG = 27, US_R_ECHO = 33;

// Encoders (single-channel)
static const uint8_t ENC_R_PIN = 16;
static const uint8_t ENC_L_PIN = 17;

// L298N motor driver
static const uint8_t MOT_ENA = 32;   // PWM left
static const uint8_t MOT_IN1 = 14;
static const uint8_t MOT_IN2 = 13;
static const uint8_t MOT_IN3 = 2;
static const uint8_t MOT_IN4 = 15;
static const uint8_t MOT_ENB = 4;    // PWM right

// Extras
static const uint8_t BUZZER_PIN = 23;

// OLED (SSD1306, 128x32 — chosen to keep redraw under 20 ms loop budget)
static const uint8_t  OLED_W    = 128;
static const uint8_t  OLED_H    = 32;
static const uint8_t  OLED_ADDR = 0x3C;

// ───────────── Robot parameters (tune to your chassis) ────────────────────
static const float WHEEL_RADIUS = 0.034f;                          // m
static const float WHEEL_BASE   = 0.150f;                          // m
static const int   ENCODER_CPR  = 20;                              // counts/rev
static const float DIST_PER_CNT = (2.0f * PI * WHEEL_RADIUS) / (float)ENCODER_CPR;

// Complementary filter: encoder dominates short-term, IMU corrects drift.
static const float ALPHA = 0.98f;

// ───────────── Timing periods (ms) ────────────────────────────────────────
static const uint32_t T_IMU_MS   = 10;    // 100 Hz IMU
static const uint32_t T_ODOM_MS  = 20;    //  50 Hz odometry
static const uint32_t T_MOTOR_MS = 20;    //  50 Hz motor apply
static const uint32_t T_TX_MS    = 50;    //  20 Hz UDP TX
static const uint32_t T_OLED_MS  = 200;   //   5 Hz OLED
static const uint32_t T_WIFI_MS  = 1000;  //   1 Hz reconnect check

// ───────────── Ultrasonic constants ───────────────────────────────────────
static const float    US_MIN_M      = 0.02f;
static const float    US_MAX_M      = 2.50f;
static const float    US_JUMP_M     = 0.50f;
static const uint32_t US_TRIG_US    = 10;
static const uint32_t US_ECHO_TO_US = 25000;       // ~4.3 m round-trip
static const uint32_t US_COOL_US    = 5000;        // gap between sensors
static const uint32_t US_STALE_MS   = 500;
static const float    US_DEFAULT_M  = US_MAX_M;    // "no obstacle" fallback

// ───────────── Motor constants ────────────────────────────────────────────
static const int PWM_MAX       = 255;
static const int PWM_DEADBAND  = 25;    // below this -> 0 (overcome stiction)
static const int PWM_RATE_STEP = 25;    // max |dPWM| per motor tick (=20 ms)

// ───────────── Safety ─────────────────────────────────────────────────────
static const uint32_t CMD_TIMEOUT_MS = 500;

// ───────────── Buzzer ─────────────────────────────────────────────────────
static const uint16_t BUZZ_ON_MS  = 80;
static const uint16_t BUZZ_GAP_MS = 80;

// ───────────── MPU6050 ────────────────────────────────────────────────────
static const uint8_t MPU_ADDR      = 0x68;
static const float   GYRO_SENS_LSB = 131.0f;          // ±250°/s -> LSB per °/s
static const float   DEG2RAD       = (float)PI / 180.0f;

// ============================================================================
// State variables
// ============================================================================
// Fused pose (single source of truth, world frame)
float gX           = 0.0f;
float gY           = 0.0f;
float gTheta       = 0.0f;     // wrapped to [-pi, pi]
float gThetaEnc    = 0.0f;     // wheel-only heading, wrapped
float gThetaImu    = 0.0f;     // gyro-only heading, wrapped
float gGyroZBias   = 0.0f;     // LSB

// Encoders — volatile because mutated in ISR
volatile uint32_t encLeftCnt  = 0;
volatile uint32_t encRightCnt = 0;
uint32_t lastEncLeft  = 0;
uint32_t lastEncRight = 0;
int8_t   lastLeftDir  = 0;
int8_t   lastRightDir = 0;

// Ultrasonic — per-sensor state
struct Sonar {
  uint8_t  trigPin;
  uint8_t  echoPin;
  volatile uint32_t echoStartUs;
  volatile uint32_t echoEndUs;
  volatile bool     edgeStart;
  volatile bool     edgeDone;
  float    distance;
  uint32_t lastGoodMs;
  bool     hasGood;
};
Sonar sonars[3] = {
  { US_F_TRIG, US_F_ECHO, 0, 0, false, false, US_DEFAULT_M, 0, false },
  { US_L_TRIG, US_L_ECHO, 0, 0, false, false, US_DEFAULT_M, 0, false },
  { US_R_TRIG, US_R_ECHO, 0, 0, false, false, US_DEFAULT_M, 0, false },
};
enum UsPhase { US_BEGIN, US_TRIG_HIGH, US_WAIT_ECHO, US_COOLDOWN };
UsPhase  usPhase   = US_BEGIN;
uint8_t  usIdx     = 0;
uint32_t usPhaseUs = 0;

// Motor commands (current = ramped, target = requested)
int targetLeftPwm   = 0;
int targetRightPwm  = 0;
int currentLeftPwm  = 0;
int currentRightPwm = 0;

// Comms
WiFiUDP udp;
StaticJsonDocument<192> rxDoc;
char txBuf[256];
char rxBuf[192];
uint32_t lastCmdMs      = 0;
bool     everReceivedCmd = false;
bool     timeoutBuzzed   = false;

// OLED
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
bool oledOk = false;

// Buzzer state machine
uint32_t buzzOnUntilMs  = 0;
uint32_t buzzNextOnAtMs = 0;
uint8_t  buzzBeepsLeft  = 0;

// Scheduler timestamps
uint32_t tImu = 0, tOdom = 0, tMot = 0, tTx = 0, tOled = 0, tWifi = 0;

// ============================================================================
// Small helpers
// ============================================================================
static inline float wrapAngle(float a) {
  while (a >  PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
static inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline int rateLimit(int target, int current, int step) {
  int d = target - current;
  if (d >  step) return current + step;
  if (d < -step) return current - step;
  return target;
}

// ============================================================================
// ISRs
// ============================================================================
void IRAM_ATTR isrEncL() { encLeftCnt++;  }
void IRAM_ATTR isrEncR() { encRightCnt++; }

static inline void onEcho(uint8_t i, uint8_t pin) {
  if (digitalRead(pin) == HIGH) {
    sonars[i].echoStartUs = micros();
    sonars[i].edgeStart   = true;
  } else if (sonars[i].edgeStart) {
    sonars[i].echoEndUs = micros();
    sonars[i].edgeDone  = true;
  }
}
void IRAM_ATTR isrEchoF() { onEcho(0, US_F_ECHO); }
void IRAM_ATTR isrEchoL() { onEcho(1, US_L_ECHO); }
void IRAM_ATTR isrEchoR() { onEcho(2, US_R_ECHO); }

// ============================================================================
// MPU6050 — raw I2C
// ============================================================================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadGyroZ(int16_t& raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  return true;
}

static void mpuInit() {
  mpuWrite(0x6B, 0x00);   // wake (clear sleep)
  mpuWrite(0x1A, 0x03);   // DLPF: ~44 Hz BW, 4.9 ms delay
  mpuWrite(0x1B, 0x00);   // GYRO_FS_SEL = 0 (±250 dps -> 131 LSB/dps)
}

static void mpuCalibrate(uint16_t samples = 400) {
  Serial.println(F("[IMU] Calibrating gyro Z bias — keep robot still..."));
  long sum = 0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t r;
    if (mpuReadGyroZ(r)) { sum += r; got++; }
    delay(3);   // setup only — not in loop()
  }
  gGyroZBias = (got > 0) ? ((float)sum / (float)got) : 0.0f;
  Serial.print(F("[IMU] Bias LSB = "));
  Serial.println(gGyroZBias, 3);
}

// ============================================================================
// Ultrasonic — non-blocking round-robin state machine
// ============================================================================
static void usValidate(uint8_t i, float meters, uint32_t nowMs) {
  if (meters < US_MIN_M || meters > US_MAX_M) return;          // OOR
  if (sonars[i].hasGood &&
      fabsf(meters - sonars[i].distance) > US_JUMP_M) return;  // jump
  sonars[i].distance   = meters;
  sonars[i].lastGoodMs = nowMs;
  sonars[i].hasGood    = true;
}

static void usTick(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  Sonar& s = sonars[usIdx];

  switch (usPhase) {
    case US_BEGIN: {
      s.edgeStart = false;
      s.edgeDone  = false;
      digitalWrite(s.trigPin, LOW);
      digitalWrite(s.trigPin, HIGH);
      usPhaseUs = nowUs;
      usPhase   = US_TRIG_HIGH;
      break;
    }
    case US_TRIG_HIGH:
      if ((nowUs - usPhaseUs) >= US_TRIG_US) {
        digitalWrite(s.trigPin, LOW);
        usPhaseUs = nowUs;
        usPhase   = US_WAIT_ECHO;
      }
      break;
    case US_WAIT_ECHO:
      if (s.edgeDone) {
        uint32_t dt = s.echoEndUs - s.echoStartUs;
        float meters = (float)dt * 0.0001715f;   // 343 m/s / 2 / 1e6
        usValidate(usIdx, meters, nowMs);
        usPhaseUs = nowUs;
        usPhase   = US_COOLDOWN;
      } else if ((nowUs - usPhaseUs) >= US_ECHO_TO_US) {
        usPhaseUs = nowUs;
        usPhase   = US_COOLDOWN;
      }
      break;
    case US_COOLDOWN:
      if ((nowUs - usPhaseUs) >= US_COOL_US) {
        usIdx   = (usIdx + 1) % 3;
        usPhase = US_BEGIN;
      }
      break;
  }

  // Stale fallback — if no fresh good reading, publish safe default.
  for (uint8_t k = 0; k < 3; k++) {
    if (sonars[k].hasGood && (nowMs - sonars[k].lastGoodMs) > US_STALE_MS) {
      sonars[k].distance = US_DEFAULT_M;
      sonars[k].hasGood  = false;
    }
  }
}

// ============================================================================
// IMU integration (100 Hz)
// ============================================================================
static void updateImu() {
  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  if (lastUs == 0) { lastUs = nowUs; return; }
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0.0f || dt > 0.25f) return;   // sanity

  int16_t raw;
  if (!mpuReadGyroZ(raw)) return;
  float dps = ((float)raw - gGyroZBias) / GYRO_SENS_LSB;   // deg/s
  float rps = dps * DEG2RAD;                                // rad/s
  gThetaImu = wrapAngle(gThetaImu + rps * dt);
}

// ============================================================================
// Odometry + IMU fusion (50 Hz)
// ============================================================================
static void updateOdometry() {
  // Atomic snapshot of encoder counts
  noInterrupts();
  uint32_t l = encLeftCnt;
  uint32_t r = encRightCnt;
  interrupts();

  uint32_t dlu = l - lastEncLeft;
  uint32_t dru = r - lastEncRight;
  lastEncLeft  = l;
  lastEncRight = r;

  // Apply commanded direction (single-channel encoders give magnitude only)
  float dsL = (float)dlu * DIST_PER_CNT * (float)lastLeftDir;
  float dsR = (float)dru * DIST_PER_CNT * (float)lastRightDir;

  float ds       = 0.5f * (dsL + dsR);
  float dthetaEn = (dsR - dsL) / WHEEL_BASE;

  gThetaEnc = wrapAngle(gThetaEnc + dthetaEn);

  // Complementary filter — wrap-safe (blend the *difference*, not raw angles)
  // Equivalent to: theta = alpha*theta_enc + (1-alpha)*theta_imu, but correct
  // across the ±pi discontinuity.
  float diff  = wrapAngle(gThetaImu - gThetaEnc);
  gTheta      = wrapAngle(gThetaEnc + (1.0f - ALPHA) * diff);

  // Position integration uses the fused heading
  gX += ds * cosf(gTheta);
  gY += ds * sinf(gTheta);
}

// ============================================================================
// Motors
// ============================================================================
static void writeMotor(uint8_t in1, uint8_t in2, uint8_t en, int pwm) {
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  } else if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  }
}

static void applyMotors() {
  // Clamp targets
  targetLeftPwm  = clampi(targetLeftPwm,  -PWM_MAX, PWM_MAX);
  targetRightPwm = clampi(targetRightPwm, -PWM_MAX, PWM_MAX);

  // Acceleration limiting (ramp toward target)
  currentLeftPwm  = rateLimit(targetLeftPwm,  currentLeftPwm,  PWM_RATE_STEP);
  currentRightPwm = rateLimit(targetRightPwm, currentRightPwm, PWM_RATE_STEP);

  // Deadband — applied AFTER rate limiting so we still cross 0 cleanly
  int outL = (abs(currentLeftPwm)  < PWM_DEADBAND) ? 0 : currentLeftPwm;
  int outR = (abs(currentRightPwm) < PWM_DEADBAND) ? 0 : currentRightPwm;

  // Track sign for encoder direction inference (preserve last when stopped)
  if (outL > 0) lastLeftDir = 1;  else if (outL < 0) lastLeftDir = -1;
  if (outR > 0) lastRightDir = 1; else if (outR < 0) lastRightDir = -1;

  writeMotor(MOT_IN1, MOT_IN2, MOT_ENA, outL);
  writeMotor(MOT_IN3, MOT_IN4, MOT_ENB, outR);
}

// ============================================================================
// UDP — TX (telemetry) and RX (commands)
// ============================================================================
static void sendTelemetry(uint32_t now) {
  // Deterministic, zero-allocation packing
  int n = snprintf(
    txBuf, sizeof(txBuf),
    "{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,"
    "\"distances\":[%.3f,%.3f,%.3f],"
    "\"timestamp\":%lu}",
    gX, gY, gTheta,
    sonars[0].distance, sonars[1].distance, sonars[2].distance,
    (unsigned long)now
  );
  if (n <= 0 || n >= (int)sizeof(txBuf)) return;
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((const uint8_t*)txBuf, n);
  udp.endPacket();
}

static void receiveCommands(uint32_t now) {
  int sz = udp.parsePacket();
  if (sz <= 0) return;
  if (sz >= (int)sizeof(rxBuf)) {
    while (udp.available()) udp.read();
    return;
  }
  int n = udp.read((uint8_t*)rxBuf, sizeof(rxBuf) - 1);
  if (n <= 0) return;
  rxBuf[n] = '\0';

  rxDoc.clear();
  DeserializationError err = deserializeJson(rxDoc, rxBuf, n);
  if (err) return;

  // Sentinel that won't be sent as a real PWM value
  const int SENT = INT32_MIN;
  int ls = rxDoc["left_speed"]  | SENT;
  int rs = rxDoc["right_speed"] | SENT;
  if (ls == SENT || rs == SENT) return;

  targetLeftPwm  = clampi(ls, -PWM_MAX, PWM_MAX);
  targetRightPwm = clampi(rs, -PWM_MAX, PWM_MAX);
  lastCmdMs      = now;

  if (!everReceivedCmd) {
    everReceivedCmd = true;
    buzzBeepsLeft   = 1;            // single beep on first ever command
    buzzNextOnAtMs  = now;
  }
  timeoutBuzzed = false;            // re-arm timeout alert
}

static void safetyWatchdog(uint32_t now) {
  if (!everReceivedCmd) return;     // not armed until first command
  if ((now - lastCmdMs) > CMD_TIMEOUT_MS) {
    targetLeftPwm  = 0;
    targetRightPwm = 0;
    if (!timeoutBuzzed) {
      buzzBeepsLeft  = 2;           // double beep on entering timeout state
      buzzNextOnAtMs = now;
      timeoutBuzzed  = true;
    }
  }
}

// ============================================================================
// Buzzer — non-blocking, queues N short beeps
// ============================================================================
static void updateBuzzer(uint32_t now) {
  // Currently sounding -> turn off when on-window expires
  if (buzzOnUntilMs != 0 && (int32_t)(now - buzzOnUntilMs) >= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzOnUntilMs = 0;
    if (buzzBeepsLeft > 0) buzzNextOnAtMs = now + BUZZ_GAP_MS;
  }
  // Silent -> start next beep if any queued and gap elapsed
  if (buzzBeepsLeft > 0 && buzzOnUntilMs == 0 &&
      (int32_t)(now - buzzNextOnAtMs) >= 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzOnUntilMs = now + BUZZ_ON_MS;
    buzzBeepsLeft--;
  }
}

// ============================================================================
// OLED — IP + pose + distances (5 Hz)
// ============================================================================
static void updateOled() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 0 — IP
  display.setCursor(0, 0);
  display.print(F("IP "));
  if (WiFi.status() == WL_CONNECTED) display.print(WiFi.localIP());
  else                                display.print(F("(no link)"));

  // Line 1 — x, y
  display.setCursor(0, 8);
  display.print(F("x"));  display.print(gX, 2);
  display.print(F(" y")); display.print(gY, 2);

  // Line 2 — theta
  display.setCursor(0, 16);
  display.print(F("t"));  display.print(gTheta, 2);
  display.print(F(" rad"));

  // Line 3 — distances F L R
  display.setCursor(0, 24);
  display.print(F("d "));
  display.print(sonars[0].distance, 2); display.print(' ');
  display.print(sonars[1].distance, 2); display.print(' ');
  display.print(sonars[2].distance, 2);

  display.display();
}

// ============================================================================
// WiFi
// ============================================================================
static void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                   // keep latency low
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("[WiFi] Connecting"));
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(250);                           // setup-time only
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] IP: ")); Serial.println(WiFi.localIP());
    udp.begin(ESP_PORT);
  } else {
    Serial.println(F("[WiFi] Connect failed — will keep retrying."));
  }
}

static void wifiCheck(uint32_t now) {
  static bool wasConnected = false;
  static uint32_t lastTry  = 0;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wasConnected) {
      udp.stop();
      udp.begin(ESP_PORT);
      Serial.print(F("[WiFi] reconnected, IP: "));
      Serial.println(WiFi.localIP());
    }
    wasConnected = true;
    return;
  }
  wasConnected = false;
  if (now - lastTry < 5000) return;
  lastTry = now;
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n=== VacBot SLAM Navigator — ESP32 boot ==="));

  // ---- Motors ----
  pinMode(MOT_IN1, OUTPUT); pinMode(MOT_IN2, OUTPUT);
  pinMode(MOT_IN3, OUTPUT); pinMode(MOT_IN4, OUTPUT);
  pinMode(MOT_ENA, OUTPUT); pinMode(MOT_ENB, OUTPUT);
  writeMotor(MOT_IN1, MOT_IN2, MOT_ENA, 0);
  writeMotor(MOT_IN3, MOT_IN4, MOT_ENB, 0);
  // Optional: lift PWM above audible range to avoid motor whine
  // analogWriteFrequency(MOT_ENA, 20000);
  // analogWriteFrequency(MOT_ENB, 20000);

  // ---- Ultrasonics ----
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(sonars[i].trigPin, OUTPUT);
    digitalWrite(sonars[i].trigPin, LOW);
    pinMode(sonars[i].echoPin, INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(US_F_ECHO), isrEchoF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_L_ECHO), isrEchoL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_R_ECHO), isrEchoR, CHANGE);

  // ---- Encoders ----
  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isrEncR, RISING);

  // ---- Buzzer ----
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // ---- I2C (MPU6050 + OLED share the bus) ----
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ---- OLED ----
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOk = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("VacBot booting..."));
    display.display();
  } else {
    Serial.println(F("[OLED] not found at 0x3C — continuing without."));
  }

  // ---- IMU ----
  mpuInit();
  delay(50);
  mpuCalibrate();

  // ---- WiFi + UDP ----
  wifiBegin();

  // ---- Scheduler ----
  uint32_t now = millis();
  tImu = tOdom = tMot = tTx = tOled = tWifi = now;
  lastCmdMs = now;   // does NOT arm the watchdog (see safetyWatchdog)

  Serial.println(F("[SYS] Ready."));
}

// ============================================================================
// LOOP — cooperative, millis-based, target <20 ms per iteration
// ============================================================================
void loop() {
  const uint32_t now = millis();

  // Per-iteration (cheap)
  usTick(now);
  receiveCommands(now);
  safetyWatchdog(now);
  updateBuzzer(now);

  // 100 Hz IMU
  if ((now - tImu) >= T_IMU_MS) {
    tImu = now;
    updateImu();
  }

  // 50 Hz odometry + fusion
  if ((now - tOdom) >= T_ODOM_MS) {
    tOdom = now;
    updateOdometry();
  }

  // 50 Hz motor apply (rate limit + PWM)
  if ((now - tMot) >= T_MOTOR_MS) {
    tMot = now;
    applyMotors();
  }

  // 20 Hz UDP telemetry
  if ((now - tTx) >= T_TX_MS) {
    tTx = now;
    if (WiFi.status() == WL_CONNECTED) sendTelemetry(now);
  }

  // 5 Hz OLED
  if ((now - tOled) >= T_OLED_MS) {
    tOled = now;
    updateOled();
  }

  // 1 Hz WiFi keep-alive
  if ((now - tWifi) >= T_WIFI_MS) {
    tWifi = now;
    wifiCheck(now);
  }
}
