/*
 * VacBot SLAM Navigator - ESP32 Main Firmware
 *
 * Responsibilities:
 *   - Sensor acquisition (MPU6050 gyro Z, 3x HC-SR04 ultrasonic, 2x wheel encoders)
 *   - Odometry + complementary IMU fusion (alpha = 0.98)
 *   - Closed-loop motor command application with acceleration limiting
 *   - UDP telemetry / command exchange with the PC SLAM stack
 *
 * Hard constraints (enforced):
 *   - Fully non-blocking (only the 10 us HC-SR04 trigger pulse uses busy-wait,
 *     as required by the sensor datasheet; nothing else blocks).
 *   - millis() / micros() based scheduling.
 *   - No dynamic allocation inside loop().
 *   - Loop iteration < 20 ms (typically < 5 ms).
 *
 * Units: meters, radians. Angles always normalized to [-pi, pi].
 * Body frame: x forward, y left, theta CCW positive.
 *
 * Required Arduino libraries:
 *   - WiFi, WiFiUdp, Wire (built-in for ESP32 core)
 *   - ArduinoJson v6.x  (Library Manager: "ArduinoJson" by Benoit Blanchon)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <math.h>

// =============================================================================
// USER CONFIGURATION  (edit before flashing)
// =============================================================================
static const char*    WIFI_SSID     = "YOUR_WIFI_SSID";
static const char*    WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char*    PC_IP         = "192.168.1.100"; // PC running the visualizer
static const uint16_t PC_PORT       = 5005;            // PC listen port
static const uint16_t LOCAL_PORT    = 5006;            // ESP32 listen port

// =============================================================================
// PIN MAP  (FIXED per spec - do not change)
// =============================================================================
static const uint8_t PIN_SDA = 21;
static const uint8_t PIN_SCL = 22;

static const uint8_t PIN_TRIG_F = 19, PIN_ECHO_F = 18;
static const uint8_t PIN_TRIG_L = 25, PIN_ECHO_L = 26;
static const uint8_t PIN_TRIG_R = 27, PIN_ECHO_R = 33;

static const uint8_t PIN_ENC_R = 16;
static const uint8_t PIN_ENC_L = 17;

static const uint8_t PIN_ENA = 32, PIN_IN1 = 14, PIN_IN2 = 13;  // motor A -> RIGHT
static const uint8_t PIN_IN3 = 2,  PIN_IN4 = 15, PIN_ENB = 4;   // motor B -> LEFT

static const uint8_t PIN_BUZZER = 23;

// =============================================================================
// ROBOT PARAMETERS  (must match calibration.ino values)
// =============================================================================
static const float WHEEL_RADIUS      = 0.03f;   // meters
static const float WHEEL_BASE        = 0.14f;   // meters
static const int   TICKS_PER_REV     = 360;
static const float COMP_ALPHA        = 0.98f;   // encoder weight in fusion

// Sensor mounting angles in body frame (radians)
static const float SENSOR_ANGLE_F =  0.0f;
static const float SENSOR_ANGLE_L =  1.5707963f;
static const float SENSOR_ANGLE_R = -1.5707963f;

// Distance filtering
static const float DIST_MIN_M  = 0.02f;
static const float DIST_MAX_M  = 2.5f;
static const float DIST_JUMP_M = 0.5f;

// PWM (ESP32 LEDC)
static const int LEDC_CH_A      = 0;
static const int LEDC_CH_B      = 1;
static const int PWM_FREQ_HZ    = 20000;
static const int PWM_RES_BITS   = 8;
static const int PWM_DEADBAND   = 25;
static const int ACCEL_STEP     = 15;   // PWM units / control tick

// Scheduling (ms)
static const uint32_t SENSOR_PERIOD_MS  = 50;   // 20 Hz odometry/fusion
static const uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz control
static const uint32_t COMM_PERIOD_MS    = 100;  // 10 Hz telemetry
static const uint32_t UDP_TIMEOUT_MS    = 500;  // stop motors on link loss
static const uint32_t US_SLOT_PERIOD_MS = 30;   // ultrasonic round-robin
static const uint32_t STALL_TIMEOUT_MS  = 1200; // wheel stall detection

// MPU6050 register map
static const uint8_t MPU_ADDR    = 0x68;
static const uint8_t MPU_PWR_MGMT = 0x6B;
static const uint8_t MPU_GYRO_CFG = 0x1B;
static const uint8_t MPU_ACC_CFG  = 0x1C;
static const uint8_t MPU_GYRO_ZH  = 0x47;

// HC-SR04 echo timeout (us). 2.5 m max -> 14.5 ms round trip; 25 ms is safe.
static const uint32_t US_TIMEOUT_US = 25000;

// =============================================================================
// STATE
// =============================================================================
struct Pose { float x; float y; float theta; };
static Pose pose = {0.0f, 0.0f, 0.0f};

// Encoder counters (modified in ISR)
static volatile long enc_left_count  = 0;
static volatile long enc_right_count = 0;
static int last_left_dir  = 0;  // sign of last commanded PWM, used by ISR
static int last_right_dir = 0;
static long prev_left_ticks  = 0;
static long prev_right_ticks = 0;

// IMU
static float gyro_z_offset = 0.0f;     // rad/s bias
static float theta_imu     = 0.0f;     // pure IMU-integrated heading
static uint32_t last_loc_us = 0;

// Ultrasonic
struct EchoSlot {
  uint8_t  trig_pin;
  uint8_t  echo_pin;
  volatile uint32_t rise_us;
  volatile uint32_t fall_us;
  volatile bool     have_pulse;
  float   last_valid;
  bool    triggered;
  uint32_t trig_time_us;
};
static EchoSlot us[3] = {
  { PIN_TRIG_F, PIN_ECHO_F, 0, 0, false, -1.0f, false, 0 },
  { PIN_TRIG_L, PIN_ECHO_L, 0, 0, false, -1.0f, false, 0 },
  { PIN_TRIG_R, PIN_ECHO_R, 0, 0, false, -1.0f, false, 0 },
};
static uint8_t us_active = 0;
static float dist_out[3] = { -1.0f, -1.0f, -1.0f };  // F, L, R

// Motors
static int cmd_left_target  = 0;
static int cmd_right_target = 0;
static int cmd_left_applied  = 0;
static int cmd_right_applied = 0;
static uint32_t last_tick_change_ms = 0;

// Networking
static WiFiUDP udp;
static uint32_t last_rx_ms      = 0;
static uint32_t last_tx_ms      = 0;
static uint32_t last_wifi_check = 0;

// Scheduler
static uint32_t last_sensor_ms  = 0;
static uint32_t last_control_ms = 0;
static uint32_t last_us_step_ms = 0;

// =============================================================================
// UTILITIES
// =============================================================================
static inline float wrap_pi(float a) {
  while (a >  M_PI) a -= 2.0f * (float)M_PI;
  while (a < -M_PI) a += 2.0f * (float)M_PI;
  return a;
}
static inline int clamp_pwm(int v) {
  if (v >  255) return  255;
  if (v < -255) return -255;
  return v;
}

// =============================================================================
// INTERRUPT HANDLERS
// =============================================================================
// Encoders are single-channel; direction is inferred from the last commanded
// motor polarity (acceptable for a slow indoor robot).
static void IRAM_ATTR isr_enc_left()  {
  if (last_left_dir  >= 0) enc_left_count++;  else enc_left_count--;
}
static void IRAM_ATTR isr_enc_right() {
  if (last_right_dir >= 0) enc_right_count++; else enc_right_count--;
}

// HC-SR04 echo edge ISRs - record rise and fall times to compute pulse width.
static void IRAM_ATTR isr_echo_front() {
  if (digitalRead(PIN_ECHO_F)) us[0].rise_us = micros();
  else { us[0].fall_us = micros(); us[0].have_pulse = true; }
}
static void IRAM_ATTR isr_echo_left()  {
  if (digitalRead(PIN_ECHO_L)) us[1].rise_us = micros();
  else { us[1].fall_us = micros(); us[1].have_pulse = true; }
}
static void IRAM_ATTR isr_echo_right() {
  if (digitalRead(PIN_ECHO_R)) us[2].rise_us = micros();
  else { us[2].fall_us = micros(); us[2].have_pulse = true; }
}

// =============================================================================
// MPU6050 (raw I2C, no external library)
// =============================================================================
static bool mpu_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpu_read_bytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
static bool mpu_begin() {
  if (!mpu_write_reg(MPU_PWR_MGMT, 0x00)) return false;  // wake
  delay(50);
  mpu_write_reg(MPU_GYRO_CFG, 0x00);   // +/- 250 deg/s, 131 LSB / (deg/s)
  mpu_write_reg(MPU_ACC_CFG,  0x00);   // +/- 2 g
  return true;
}
static bool mpu_read_gyro_z(float* gz_rad_s) {
  uint8_t b[2];
  if (!mpu_read_bytes(MPU_GYRO_ZH, b, 2)) return false;
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  float gz_dps = (float)raw / 131.0f;
  // Spec convention: CCW positive. If your board sees the opposite sign,
  // flip here:  *gz_rad_s = -gz_dps * (M_PI / 180.0f);
  *gz_rad_s = gz_dps * ((float)M_PI / 180.0f);
  return true;
}
static void mpu_calibrate_gyro(uint16_t samples) {
  double sum = 0.0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < samples; i++) {
    float g;
    if (mpu_read_gyro_z(&g)) { sum += g; got++; }
    delay(3);
  }
  gyro_z_offset = (got > 0) ? (float)(sum / got) : 0.0f;
}

// =============================================================================
// MOTOR DRIVER (L298N)
// =============================================================================
static void motor_init() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  ledcSetup(LEDC_CH_A, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(LEDC_CH_B, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_ENA, LEDC_CH_A);
  ledcAttachPin(PIN_ENB, LEDC_CH_B);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  ledcWrite(LEDC_CH_A, 0);
  ledcWrite(LEDC_CH_B, 0);
}
static void apply_right_pwm(int p) {
  p = clamp_pwm(p);
  if (abs(p) < PWM_DEADBAND) p = 0;
  if (p > 0)      last_right_dir =  1;
  else if (p < 0) last_right_dir = -1;
  if (p >= 0) {
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    ledcWrite(LEDC_CH_A, p);
  } else {
    digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
    ledcWrite(LEDC_CH_A, -p);
  }
}
static void apply_left_pwm(int p) {
  p = clamp_pwm(p);
  if (abs(p) < PWM_DEADBAND) p = 0;
  if (p > 0)      last_left_dir =  1;
  else if (p < 0) last_left_dir = -1;
  if (p >= 0) {
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    ledcWrite(LEDC_CH_B, p);
  } else {
    digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
    ledcWrite(LEDC_CH_B, -p);
  }
}
static void motors_full_stop() {
  cmd_left_target = cmd_right_target = 0;
  cmd_left_applied = cmd_right_applied = 0;
  apply_left_pwm(0);
  apply_right_pwm(0);
}

// =============================================================================
// ULTRASONIC NON-BLOCKING ROUND-ROBIN
// =============================================================================
static void us_trigger(EchoSlot& s) {
  s.have_pulse  = false;
  s.triggered   = true;
  s.trig_time_us = micros();
  digitalWrite(s.trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trig_pin, HIGH);
  delayMicroseconds(10);            // 10 us trigger pulse required by HC-SR04
  digitalWrite(s.trig_pin, LOW);
}
static void us_finish(EchoSlot& s, float& out) {
  if (!s.triggered) return;
  bool done = false;
  if (s.have_pulse) {
    uint32_t dt = s.fall_us - s.rise_us;
    if (dt > 0 && dt < US_TIMEOUT_US) {
      float d = (dt * 0.000343f) * 0.5f;      // speed of sound ~343 m/s
      if (d >= DIST_MIN_M && d <= DIST_MAX_M) {
        if (s.last_valid < 0.0f || fabsf(d - s.last_valid) < DIST_JUMP_M) {
          s.last_valid = d;
          out          = d;
        }
      }
    }
    done = true;
  } else if (micros() - s.trig_time_us > US_TIMEOUT_US) {
    // No echo received in time -> out-of-range. Leave out as previous value
    // but mark slot for retrigger; we do NOT poison the map with a bogus hit.
    done = true;
  }
  if (done) s.triggered = false;
}
static void us_step(uint32_t now_ms) {
  if (now_ms - last_us_step_ms < US_SLOT_PERIOD_MS) return;
  last_us_step_ms = now_ms;
  us_finish(us[us_active], dist_out[us_active]);
  us_active = (us_active + 1) % 3;
  us_trigger(us[us_active]);
}

// =============================================================================
// LOCALIZATION (odometry + complementary IMU fusion)
// =============================================================================
static void update_localization(float dt) {
  // snapshot encoder counts
  noInterrupts();
  long el = enc_left_count;
  long er = enc_right_count;
  interrupts();

  long dl_ticks = el - prev_left_ticks;
  long dr_ticks = er - prev_right_ticks;
  prev_left_ticks  = el;
  prev_right_ticks = er;

  // Stall watchdog
  if (dl_ticks != 0 || dr_ticks != 0) last_tick_change_ms = millis();

  const float circumference = 2.0f * (float)M_PI * WHEEL_RADIUS;
  float dl = circumference * ((float)dl_ticks / (float)TICKS_PER_REV);
  float dr = circumference * ((float)dr_ticks / (float)TICKS_PER_REV);
  float ds       = 0.5f * (dl + dr);
  float dtheta_e = (dr - dl) / WHEEL_BASE;

  // IMU integration
  float gz;
  if (mpu_read_gyro_z(&gz)) {
    gz -= gyro_z_offset;
    // Clamp to physically plausible rate (saturation guard)
    if (gz >  8.0f) gz =  8.0f;
    if (gz < -8.0f) gz = -8.0f;
    theta_imu = wrap_pi(theta_imu + gz * dt);
  }

  // Complementary fusion on theta:
  //   theta_new = alpha * (theta_old + dtheta_enc) + (1-alpha) * theta_imu
  // Implemented as wrap-safe additive form.
  float diff = wrap_pi(theta_imu - pose.theta);
  pose.theta = wrap_pi(pose.theta + COMP_ALPHA * dtheta_e + (1.0f - COMP_ALPHA) * diff);

  // World-frame translation (x forward, y left)
  pose.x += ds * cosf(pose.theta);
  pose.y += ds * sinf(pose.theta);
}

// =============================================================================
// CONTROL (acceleration-limited application of incoming PWM targets)
// =============================================================================
static void apply_acceleration_limit(int& applied, int target) {
  if      (applied < target) applied = min(applied + ACCEL_STEP, target);
  else if (applied > target) applied = max(applied - ACCEL_STEP, target);
}
static void update_control() {
  apply_acceleration_limit(cmd_left_applied,  clamp_pwm(cmd_left_target));
  apply_acceleration_limit(cmd_right_applied, clamp_pwm(cmd_right_target));
  apply_left_pwm(cmd_left_applied);
  apply_right_pwm(cmd_right_applied);
}

// =============================================================================
// COMMUNICATION
// =============================================================================
static void send_telemetry() {
  StaticJsonDocument<256> doc;
  doc["x"]     = pose.x;
  doc["y"]     = pose.y;
  doc["theta"] = pose.theta;
  JsonArray d  = doc.createNestedArray("distances");
  d.add(dist_out[0]);
  d.add(dist_out[1]);
  d.add(dist_out[2]);
  doc["timestamp"] = (uint32_t)millis();
  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}
static void poll_commands() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;
  char buf[160];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, buf) != DeserializationError::Ok) return;
  cmd_left_target  = clamp_pwm((int)(doc["left_speed"]  | 0));
  cmd_right_target = clamp_pwm((int)(doc["right_speed"] | 0));
  last_rx_ms = millis();
}
static void wifi_ensure() {
  uint32_t now = millis();
  if (now - last_wifi_check < 1000) return;
  last_wifi_check = now;
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// =============================================================================
// SAFETY
// =============================================================================
static void safety_check() {
  uint32_t now = millis();
  // UDP timeout -> halt
  if (now - last_rx_ms > UDP_TIMEOUT_MS) {
    cmd_left_target  = 0;
    cmd_right_target = 0;
  }
  // Encoder stall -> halt (only when actively commanded)
  if ((abs(cmd_left_applied) > PWM_DEADBAND || abs(cmd_right_applied) > PWM_DEADBAND)
      && (now - last_tick_change_ms > STALL_TIMEOUT_MS)) {
    cmd_left_target = 0; cmd_right_target = 0;
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Ultrasonic pins
  pinMode(PIN_TRIG_F, OUTPUT); pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_TRIG_L, OUTPUT); pinMode(PIN_ECHO_L, INPUT);
  pinMode(PIN_TRIG_R, OUTPUT); pinMode(PIN_ECHO_R, INPUT);
  digitalWrite(PIN_TRIG_F, LOW);
  digitalWrite(PIN_TRIG_L, LOW);
  digitalWrite(PIN_TRIG_R, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_F), isr_echo_front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_L), isr_echo_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_R), isr_echo_right, CHANGE);

  // Encoders
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isr_enc_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isr_enc_right, RISING);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Motors
  motor_init();

  // IMU
  if (mpu_begin()) {
    Serial.println(F("[INIT] MPU6050 OK. Calibrating gyro (~0.6 s)... hold still."));
    mpu_calibrate_gyro(200);
    Serial.print(F("[INIT] gyro_z_offset (rad/s) = "));
    Serial.println(gyro_z_offset, 6);
  } else {
    Serial.println(F("[WARN] MPU6050 init failed; continuing on encoder-only odometry."));
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(50);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[INIT] WiFi connected. IP = "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WARN] WiFi not yet connected; will keep retrying."));
  }
  udp.begin(LOCAL_PORT);

  // Start ultrasonic chain
  us_trigger(us[us_active]);

  // Init schedulers
  uint32_t now = millis();
  last_loc_us         = micros();
  last_sensor_ms      = now;
  last_control_ms     = now;
  last_tx_ms          = now;
  last_us_step_ms     = now;
  last_wifi_check     = now;
  last_rx_ms          = now;          // give a grace period at boot
  last_tick_change_ms = now;
}

void loop() {
  uint32_t now = millis();

  // Network housekeeping (cheap; called every loop)
  wifi_ensure();
  poll_commands();

  // Ultrasonic round-robin advance
  us_step(now);

  // 20 Hz: localization update
  if (now - last_sensor_ms >= SENSOR_PERIOD_MS) {
    last_sensor_ms = now;
    uint32_t now_us = micros();
    float dt = (now_us - last_loc_us) * 1e-6f;
    last_loc_us = now_us;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.05f;
    update_localization(dt);
  }

  // 50 Hz: safety + motor command application
  if (now - last_control_ms >= CONTROL_PERIOD_MS) {
    last_control_ms = now;
    safety_check();
    update_control();
  }

  // 10 Hz: telemetry to PC
  if (now - last_tx_ms >= COMM_PERIOD_MS) {
    last_tx_ms = now;
    if (WiFi.status() == WL_CONNECTED) send_telemetry();
  }
}
