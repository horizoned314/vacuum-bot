// =============================================================================
//  main.ino — VacBot SLAM Navigator  (ESP32 firmware, networked role)
//  Board  : DOIT ESP32 DEVKIT V1
//  Core   : ESP32 Arduino Core v3.8.8  (ledcAttach / ledcWrite by PIN)
//  Role   : Sensor + actuator slave.  PC runs SLAM, A*, and the control law.
//
//  Hardware contract (verified against calibration.ino — DO NOT change without
//  re-checking the wiring):
//    HC-SR04 : TRIG_F=19 ECHO_F=18  TRIG_L=25 ECHO_L=26  TRIG_R=27 ECHO_R=33
//    Encoders: ENC_L=35  ENC_R=34   (input-only pins, no pull-up available)
//    L298N   : ENA=32 IN1=14 IN2=13  → drives LEFT  motor
//              ENB= 4 IN3= 2 IN4=15  → drives RIGHT motor
//    I2C     : SDA=21  SCL=22        (MPU6050 + SSD1306)
//    Buzzer  : 23
//    OLED    : SSD1306 128×32 @ 0x3C
//
//  Calibration constants (taken from your actual run output):
//    motorBalanceFactor = countL / countR = 0.776  → right motor is faster;
//                                                    right PWM is scaled ×0.776
//    GZ bias ≈ 0 LSB after warm-up  → we still recalibrate at boot for safety.
//
//  Frame convention (matches README §2 and pc/slam.py):
//    x forward (robot nose)   y left   θ CCW positive   angles wrapped to (−π, π]
//
//  Wire protocol (matches README §2 and pc/network.py):
//    Robot → PC  (10 Hz):
//      {"x":<m>,"y":<m>,"theta":<rad>,"distances":[F,L,R],"timestamp":<ms>}
//      Distances: meters in [0.02, 2.50].  -1.0 = invalid for that frame
//                 (out of range, or frame-to-frame jump > 0.5 m → rejected).
//    PC → Robot:
//      {"left_speed":<-255..255>,"right_speed":<-255..255>}
//      If no command received for 500 ms, motors stop until packets resume.
//
//  Libraries (install via Library Manager):
//    • ArduinoJson         v7.x  (Benoît Blanchon)
//    • Adafruit SSD1306    + Adafruit GFX
//    • MPU6050 by Electronic Cats   (or i2cdevlib jrowberg version)
// =============================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"
#include <math.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
//  USER CONFIGURATION  ← edit these for your network
// ─────────────────────────────────────────────────────────────────────────────
const char*    WIFI_SSID   = "YOUR_WIFI";
const char*    WIFI_PASS   = "YOUR_PASSWORD";
IPAddress      PC_IP       (192, 168, 1, 100);   // host PC IP
const uint16_t PC_PORT      = 5005;              // PC's listen port
const uint16_t LOCAL_PORT   = 5006;              // ESP32's listen port

// If your IMU's GZ sign turns out reversed (rotate left → θ decreases),
// flip this to -1.0f.  Verify with the calibration sketch first.
constexpr float GYRO_Z_SIGN = +1.0f;

// ─────────────────────────────────────────────────────────────────────────────
//  PIN MAP  (from calibration.ino)
// ─────────────────────────────────────────────────────────────────────────────
#define TRIG_F  19
#define ECHO_F  18
#define TRIG_L  25
#define ECHO_L  26
#define TRIG_R  27
#define ECHO_R  33

#define ENC_L_PIN 35
#define ENC_R_PIN 34

#define IN1 14   // LEFT  forward / reverse
#define IN2 13
#define ENA 32   // LEFT  PWM
#define IN3  2   // RIGHT forward / reverse
#define IN4 15
#define ENB  4   // RIGHT PWM

#define PIN_SDA 21
#define PIN_SCL 22
#define BUZZER  23

// ─────────────────────────────────────────────────────────────────────────────
//  OLED
// ─────────────────────────────────────────────────────────────────────────────
#define SCREEN_W 128
#define SCREEN_H  32
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oled_ok = false;

// ─────────────────────────────────────────────────────────────────────────────
//  IMU
// ─────────────────────────────────────────────────────────────────────────────
MPU6050 mpu;
float gyroOffsetZ = 0.0f;   // raw LSB bias

// ─────────────────────────────────────────────────────────────────────────────
//  ROBOT PHYSICAL CONSTANTS
//  ⚠ Re-verify TICKS_PER_REV using calibration.ino if you change encoders.
// ─────────────────────────────────────────────────────────────────────────────
constexpr float WHEEL_R       = 0.033f;   // wheel radius (m)
constexpr float WHEEL_BASE    = 0.145f;   // wheel-to-wheel track (m)
constexpr float TICKS_PER_REV = 20.0f;    // encoder pulses per full wheel turn
constexpr float DIST_PER_TICK = (2.0f * (float)M_PI * WHEEL_R) / TICKS_PER_REV;

// Calibration output: motorBalanceFactor = countL / countR = 0.776
// Right motor spins faster → right PWM is scaled down so straight drive
// produces equal wheel distances.
constexpr float MOTOR_BALANCE = 0.776f;

constexpr float CF_ALPHA = 0.98f;   // 0.98 odom heading, 0.02 IMU correction

// ─────────────────────────────────────────────────────────────────────────────
//  ULTRASONIC  (interrupt-based, non-blocking)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float    SONIC_MIN_M     = 0.02f;
constexpr float    SONIC_MAX_M     = 2.50f;
constexpr float    SONIC_JUMP_M    = 0.50f;     // reject frame-to-frame jumps > this
constexpr uint32_t ECHO_TIMEOUT_US = 14710UL;   // 2.5 m @ 340 m/s × 2

struct EchoSlot {
  uint8_t           trig_pin, echo_pin;
  volatile uint32_t rise_us, fall_us;
  volatile bool     have_pulse;
  bool              triggered;
  uint32_t          trig_at_us;
  float             dist_m;          // last accepted reading (m)
  bool              valid;           // is dist_m fresh & in-range?
};
EchoSlot us[3];      // 0 = front, 1 = left, 2 = right
uint8_t  us_active = 0;

// Body-frame mounting angles — MUST match SENSOR_ANGLES in pc/main.py.
constexpr float BEAM[3] = { 0.0f, (float)M_PI / 2.0f, -(float)M_PI / 2.0f };

// ─────────────────────────────────────────────────────────────────────────────
//  LOCALIZATION
// ─────────────────────────────────────────────────────────────────────────────
volatile long enc_l = 0, enc_r = 0;
long prev_enc_l = 0, prev_enc_r = 0;
volatile int8_t dir_l = 1, dir_r = 1;   // updated by motor driver, read by ISRs

float    pose_x = 0.0f, pose_y = 0.0f, pose_theta = 0.0f;
float    imu_theta = 0.0f;
uint32_t loc_prev_us = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR  — PC sends targets, we ramp toward them
// ─────────────────────────────────────────────────────────────────────────────
constexpr int DEADBAND = 28;    // minimum effective PWM (from calibration)
constexpr int ACCEL    = 18;    // max PWM change per 20 ms control tick

int tgt_l = 0, tgt_r = 0;       // current target (from PC)
int cur_l = 0, cur_r = 0;       // ramped actual output

// ─────────────────────────────────────────────────────────────────────────────
//  COMMS
// ─────────────────────────────────────────────────────────────────────────────
WiFiUDP  udp;
uint32_t last_cmd_ms = 0;       // time of last valid PC command
bool     pc_alive    = false;   // false ⇒ motors held at 0
constexpr uint32_t CMD_TIMEOUT_MS = 500;

uint32_t tx_count = 0, rx_count = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  SCHEDULER
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint32_t DT_SENSE = 50;     // 20 Hz: IMU + odom integration
constexpr uint32_t DT_CTRL  = 20;     // 50 Hz: motor ramp
constexpr uint32_t DT_TX    = 100;    // 10 Hz: telemetry
constexpr uint32_t DT_SONIC = 30;     // ~33 Hz: ultrasonic round-robin
constexpr uint32_t DT_OLED  = 350;    //  3 Hz: OLED redraw
constexpr uint32_t DT_WIFI  = 2000;   // 0.5 Hz: WiFi watchdog

uint32_t t_sense = 0, t_ctrl = 0, t_tx = 0, t_sonic = 0, t_oled = 0, t_wifi = 0;

// =============================================================================
//  UTILITY
// =============================================================================
static inline float wrap_pi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

static void beep(int freq, int ms) { tone(BUZZER, freq, ms); }

// =============================================================================
//  ISRs
// =============================================================================
// Single-channel encoders: direction inferred from last commanded PWM sign.
void IRAM_ATTR isr_enc_l() { enc_l += dir_l; }
void IRAM_ATTR isr_enc_r() { enc_r += dir_r; }

void IRAM_ATTR isr_echo_f() {
  if (digitalRead(ECHO_F)) us[0].rise_us = micros();
  else { us[0].fall_us = micros(); us[0].have_pulse = true; }
}
void IRAM_ATTR isr_echo_l() {
  if (digitalRead(ECHO_L)) us[1].rise_us = micros();
  else { us[1].fall_us = micros(); us[1].have_pulse = true; }
}
void IRAM_ATTR isr_echo_r() {
  if (digitalRead(ECHO_R)) us[2].rise_us = micros();
  else { us[2].fall_us = micros(); us[2].have_pulse = true; }
}

// =============================================================================
//  ULTRASONIC trigger + round-robin step
// =============================================================================
static void us_trigger(uint8_t idx) {
  EchoSlot &s = us[idx];
  s.have_pulse = false;
  s.triggered  = true;
  s.trig_at_us = micros();
  digitalWrite(s.trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trig_pin, HIGH);
  delayMicroseconds(10);   // mandatory HC-SR04 trigger pulse
  digitalWrite(s.trig_pin, LOW);
}

static void us_step() {
  EchoSlot &s = us[us_active];

  if (s.triggered) {
    bool timed_out = (micros() - s.trig_at_us) > ECHO_TIMEOUT_US;

    if (s.have_pulse) {
      uint32_t dt = s.fall_us - s.rise_us;
      bool accepted = false;
      if (dt > 0 && dt < ECHO_TIMEOUT_US) {
        float d = dt * 1e-6f * 340.0f * 0.5f;
        if (d >= SONIC_MIN_M && d <= SONIC_MAX_M) {
          // Reject impossible jumps (per README §2 filtering policy)
          if (!s.valid || fabsf(d - s.dist_m) < SONIC_JUMP_M) {
            s.dist_m  = d;
            s.valid   = true;
            accepted  = true;
          }
        }
      }
      // If we received a pulse but it was rejected, mark invalid for this frame
      if (!accepted) s.valid = false;
      s.triggered = false;
    } else if (timed_out) {
      // No echo within 2.5 m round trip → mark invalid (open space / miss)
      s.valid     = false;
      s.triggered = false;
    } else {
      return;   // still waiting for echo
    }
  }

  us_active = (us_active + 1) % 3;
  us_trigger(us_active);
}

// =============================================================================
//  IMU  (matches calibration.ino library calls)
// =============================================================================
static void calibrate_gyro(int samples = 500) {
  Serial.print(F("[MPU] Calibrating gyro (keep still)..."));
  double sum = 0;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(4);
  }
  gyroOffsetZ = (float)(sum / samples);
  Serial.printf(" offset=%.1f LSB\n", gyroOffsetZ);
}

static float read_gz_rads() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float gz_dps = (gz - gyroOffsetZ) / 131.0f;
  return GYRO_Z_SIGN * gz_dps * ((float)M_PI / 180.0f);
}

// =============================================================================
//  ODOMETRY + COMPLEMENTARY FILTER
// =============================================================================
static void localization_update() {
  uint32_t now_us = micros();
  float dt = (now_us - loc_prev_us) * 1e-6f;
  loc_prev_us = now_us;
  if (dt <= 0.0f || dt > 0.3f) return;

  noInterrupts();
  long el = enc_l, er = enc_r;
  interrupts();

  long dtl = el - prev_enc_l;
  long dtr = er - prev_enc_r;
  prev_enc_l = el;
  prev_enc_r = er;

  float dl  = dtl * DIST_PER_TICK;
  float dr  = dtr * DIST_PER_TICK;
  float dc  = (dl + dr) * 0.5f;
  float dth = (dr - dl) / WHEEL_BASE;

  float gz   = read_gz_rads();
  imu_theta  = wrap_pi(imu_theta + gz * dt);

  // Complementary filter: trust odometry rate, correct drift with IMU
  float diff = wrap_pi(imu_theta - pose_theta);
  pose_theta = wrap_pi(pose_theta + CF_ALPHA * dth + (1.0f - CF_ALPHA) * diff);

  // Midpoint integration for position
  float mid_th = pose_theta - dth * 0.5f;
  pose_x += dc * cosf(mid_th);
  pose_y += dc * sinf(mid_th);
}

// =============================================================================
//  MOTOR CONTROL
// =============================================================================
static void apply_left(int pwm) {
  pwm = constrain(pwm, -255, 255);
  dir_l = (pwm >= 0) ? 1 : -1;
  if      (pwm > 0)  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (pwm < 0)  { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
  ledcWrite(ENA, (pwm == 0) ? 0 : max(DEADBAND, abs(pwm)));
}

static void apply_right(int pwm) {
  pwm = constrain(pwm, -255, 255);
  int pwm_bal = (int)(pwm * MOTOR_BALANCE);   // scale faster motor down
  pwm_bal = constrain(pwm_bal, -255, 255);
  dir_r = (pwm >= 0) ? 1 : -1;
  if      (pwm_bal > 0)  { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (pwm_bal < 0)  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else                   { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
  ledcWrite(ENB, (pwm_bal == 0) ? 0 : max(DEADBAND, abs(pwm_bal)));
}

static void motors_stop_now() {
  tgt_l = tgt_r = cur_l = cur_r = 0;
  apply_left(0); apply_right(0);
}

static void motor_tick() {
  if (!pc_alive) {
    cur_l = cur_r = 0;
    apply_left(0); apply_right(0);
    return;
  }
  cur_l += constrain(tgt_l - cur_l, -ACCEL, ACCEL);
  cur_r += constrain(tgt_r - cur_r, -ACCEL, ACCEL);
  apply_left(cur_l);
  apply_right(cur_r);
}

// =============================================================================
//  WiFi + UDP
// =============================================================================
static void wifi_connect_blocking(uint32_t timeout_ms = 15000) {
  Serial.printf("[WiFi] Connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);    // keep latency low
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] Connected.  IP = "));
    Serial.println(WiFi.localIP());
    udp.begin(LOCAL_PORT);
    Serial.printf("[UDP ] Listening on %u.  Sending to ", LOCAL_PORT);
    Serial.print(PC_IP); Serial.printf(":%u\n", PC_PORT);
    beep(2200, 80);
  } else {
    Serial.println(F("[WiFi] Failed — will retry in loop()"));
    beep(400, 200);
  }
}

static void wifi_watchdog() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println(F("[WiFi] Disconnected — reconnecting..."));
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Connection completes asynchronously; on next watchdog tick we re-check.
}

// Build & send telemetry JSON
static void send_telemetry() {
  if (WiFi.status() != WL_CONNECTED) return;

  JsonDocument doc;
  doc["x"]         = pose_x;
  doc["y"]         = pose_y;
  doc["theta"]     = pose_theta;
  JsonArray d      = doc["distances"].to<JsonArray>();
  d.add(us[0].valid ? us[0].dist_m : -1.0f);
  d.add(us[1].valid ? us[1].dist_m : -1.0f);
  d.add(us[2].valid ? us[2].dist_m : -1.0f);
  doc["timestamp"] = (uint32_t)millis();

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
  tx_count++;
}

// Drain any pending command packets; keep only the latest
static void poll_commands() {
  int sz;
  bool got_any = false;
  int  latest_l = 0, latest_r = 0;

  while ((sz = udp.parsePacket()) > 0) {
    char buf[256];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = '\0';

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf);
    if (err) continue;
    if (!doc["left_speed"].is<int>() || !doc["right_speed"].is<int>()) continue;

    latest_l = doc["left_speed"].as<int>();
    latest_r = doc["right_speed"].as<int>();
    got_any  = true;
    rx_count++;
  }

  if (got_any) {
    tgt_l       = constrain(latest_l, -255, 255);
    tgt_r       = constrain(latest_r, -255, 255);
    last_cmd_ms = millis();
    if (!pc_alive) { pc_alive = true; beep(1800, 40); }
  }
}

static void comms_safety_check() {
  if (pc_alive && (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    pc_alive = false;
    tgt_l = tgt_r = 0;
    Serial.println(F("[UDP ] Timeout — motors stopped."));
    beep(600, 60);
  }
}

// =============================================================================
//  OLED
// =============================================================================
static void oled_update() {
  if (!oled_ok) return;
  display.clearDisplay();
  display.setCursor(0, 0);

  // Line 0: link status + RX/TX counts (last 3 digits)
  if (WiFi.status() == WL_CONNECTED) {
    display.printf("WiFi:OK PC:%s\n", pc_alive ? "ON " : "off");
  } else {
    display.printf("WiFi:-- PC:--\n");
  }
  // Line 1: pose
  display.printf("X%+5.2f Y%+5.2f T%+4.0f\n",
                 pose_x, pose_y, pose_theta * 180.0f / (float)M_PI);
  // Line 2: distances (cm), '-' for invalid
  auto fmt = [](const EchoSlot& s) -> int { return s.valid ? (int)(s.dist_m * 100.0f) : -1; };
  display.printf("F%3d L%3d R%3d cm\n", fmt(us[0]), fmt(us[1]), fmt(us[2]));
  // Line 3: command + encoder snapshot
  display.printf("L%+4d R%+4d EL%ld\n", cur_l, cur_r, enc_l);
  display.display();
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n[VacBot] Booting (network role)..."));

  // I²C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // OLED
  oled_ok = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oled_ok) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("VacBot booting..."));
    display.display();
  } else {
    Serial.println(F("[WARN] OLED not found."));
  }

  // Buzzer
  pinMode(BUZZER, OUTPUT);
  beep(1000, 120);

  // MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("[MPU ] OK"));
    calibrate_gyro(500);
  } else {
    Serial.println(F("[WARN] MPU6050 not found — heading from encoder only."));
  }
  loc_prev_us = micros();

  // HC-SR04 pins + echo ISRs
  const uint8_t TRIGS[3] = { TRIG_F, TRIG_L, TRIG_R };
  const uint8_t ECHOS[3] = { ECHO_F, ECHO_L, ECHO_R };
  for (int i = 0; i < 3; i++) {
    us[i] = { TRIGS[i], ECHOS[i], 0, 0, false, false, 0, 0.0f, false };
    pinMode(TRIGS[i], OUTPUT);
    digitalWrite(TRIGS[i], LOW);
    pinMode(ECHOS[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ECHO_F), isr_echo_f, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), isr_echo_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), isr_echo_r, CHANGE);

  // Encoders (GPIO 34/35 = input-only, no PULLUP)
  pinMode(ENC_L_PIN, INPUT);
  pinMode(ENC_R_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isr_enc_r, RISING);

  // L298N (ESP32 v3.x LEDC API: ledcAttach by PIN)
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  motors_stop_now();

  // WiFi (blocking once, then watchdog handles reconnects)
  wifi_connect_blocking();

  // First sonic trigger to bootstrap the round-robin
  us_trigger(0);

  uint32_t now = millis();
  t_sense = t_ctrl = t_tx = t_sonic = t_oled = t_wifi = now;

  Serial.println(F("[VacBot] Ready.  Awaiting PC commands."));
  beep(2000, 80);
}

// =============================================================================
//  LOOP — cooperative scheduler, no delay()
// =============================================================================
void loop() {
  uint32_t now = millis();

  // ~33 Hz: ultrasonic round-robin
  if (now - t_sonic >= DT_SONIC) {
    t_sonic = now;
    us_step();
  }

  // 20 Hz: IMU + odometry
  if (now - t_sense >= DT_SENSE) {
    t_sense = now;
    localization_update();
  }

  // Always drain incoming command packets (cheap)
  poll_commands();
  comms_safety_check();

  // 50 Hz: motor ramp
  if (now - t_ctrl >= DT_CTRL) {
    t_ctrl = now;
    motor_tick();
  }

  // 10 Hz: telemetry TX
  if (now - t_tx >= DT_TX) {
    t_tx = now;
    send_telemetry();
  }

  // 3 Hz: OLED
  if (now - t_oled >= DT_OLED) {
    t_oled = now;
    oled_update();
  }

  // 0.5 Hz: WiFi watchdog
  if (now - t_wifi >= DT_WIFI) {
    t_wifi = now;
    wifi_watchdog();
  }
}
