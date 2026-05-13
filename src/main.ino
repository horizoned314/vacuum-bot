// =============================================================================
// VacBot SLAM Navigator — ESP32 Main Firmware
// =============================================================================
// Hardware:
//   ESP32 DevKit V1, MPU6050, 3x HC-SR04, 2x DC motors + encoders,
//   L298N motor driver, SSD1306 OLED, Active buzzer, 2x 18650 + buck
//
// Coordinate frame: x=forward, y=left, theta=CCW from x-axis (radians)
// Units: meters, radians
// All loops are non-blocking (millis()-based). No delay() calls.
// =============================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <math.h>

// ---------------------------------------------------------------------------
// WIFI / UDP CONFIGURATION  — edit before flashing
// ---------------------------------------------------------------------------
const char* WIFI_SSID     = "YourSSID";
const char* WIFI_PASSWORD = "YourPassword";
const char* PC_IP         = "192.168.1.100";   // PC running Python system
const uint16_t ROBOT_UDP_PORT = 5005;
const uint16_t PC_UDP_PORT    = 5006;

// ---------------------------------------------------------------------------
// PIN ASSIGNMENTS
// ---------------------------------------------------------------------------

// --- HC-SR04 (non-blocking trigger/echo) ---
#define TRIG_FRONT  12
#define ECHO_FRONT  14
#define TRIG_LEFT   27
#define ECHO_LEFT   26
#define TRIG_RIGHT  25
#define ECHO_RIGHT  33

// --- Motor Left (L298N) ---
#define MOTOR_L_IN1  32
#define MOTOR_L_IN2  35   // input-only pin — swap if needed
#define MOTOR_L_PWM  34   // input-only — swap if needed
// NOTE: On DevKit V1, GPIO 34/35 are input-only.
// Reassign to output-capable pins on your board:
#undef MOTOR_L_IN2
#undef MOTOR_L_PWM
#define MOTOR_L_IN2  15
#define MOTOR_L_PWM  2

// --- Motor Right (L298N) ---
#define MOTOR_R_IN1  4
#define MOTOR_R_IN2  16
#define MOTOR_R_PWM  17

// --- Encoder Left ---
#define ENC_L_A  18
#define ENC_L_B  19

// --- Encoder Right ---
#define ENC_R_A  22
#define ENC_R_B  23

// --- MPU6050 I2C ---
#define MPU_ADDR  0x68
#define SDA_PIN   21
#define SCL_PIN   22   // I2C shares with encoder R_A — reassign if conflict
// In production, use SDA=21, SCL=22, encoder on different pins.
// Reassign ENC_R_A to pin 5 to avoid conflict:
#undef ENC_R_A
#define ENC_R_A  5

// --- SSD1306 OLED ---
#define OLED_ADDR   0x3C
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// --- Buzzer ---
#define BUZZER_PIN  13

// --- PWM channels ---
#define PWM_FREQ        1000
#define PWM_RESOLUTION  8     // 8-bit: 0–255
#define PWM_CH_LEFT     0
#define PWM_CH_RIGHT    1

// ---------------------------------------------------------------------------
// ROBOT PHYSICAL PARAMETERS (tune during calibration)
// ---------------------------------------------------------------------------
const float WHEEL_RADIUS      = 0.033f;   // meters
const float WHEEL_BASE        = 0.145f;   // meters (track width between wheels)
const float TICKS_PER_REV     = 20.0f;    // encoder ticks per full wheel rotation
const float DIST_PER_TICK     = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ---------------------------------------------------------------------------
// COMPLEMENTARY FILTER
// ---------------------------------------------------------------------------
const float ALPHA = 0.98f;   // weight for odometry heading (vs IMU)

// ---------------------------------------------------------------------------
// ULTRASONIC CONSTRAINTS
// ---------------------------------------------------------------------------
const float SONIC_MAX_RANGE = 2.5f;         // meters
const float SONIC_TIMEOUT_US = 14706.0f;    // 2.5 m * 2 / 340 m/s in µs
const float SONIC_NOISE_M = 0.02f;

// ---------------------------------------------------------------------------
// TIMING INTERVALS (milliseconds)
// ---------------------------------------------------------------------------
const uint32_t INTERVAL_SENSOR  = 50;   // 20 Hz
const uint32_t INTERVAL_COMM    = 50;   // 20 Hz
const uint32_t INTERVAL_CONTROL = 20;   // 50 Hz
const uint32_t INTERVAL_OLED    = 500;  // 2 Hz (OLED is slow)
const uint32_t CMD_TIMEOUT_MS   = 1000; // error beep if no command

// ---------------------------------------------------------------------------
// OBJECTS
// ---------------------------------------------------------------------------
WiFiUDP udp;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

// --- Pose ---
volatile float pose_x     = 0.0f;
volatile float pose_y     = 0.0f;
volatile float pose_theta = 0.0f;

// --- Odometry internals ---
volatile long enc_left  = 0;
volatile long enc_right = 0;
long prev_enc_left  = 0;
long prev_enc_right = 0;

// --- IMU ---
float gyro_z_bias   = 0.0f;
float imu_theta     = 0.0f;
uint32_t imu_last_us = 0;

// Raw MPU6050 registers
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
const float GYRO_SCALE = 131.0f;     // LSB/(deg/s) for ±250 dps
const float DEG_TO_RAD = M_PI / 180.0f;

// --- Ultrasonics ---
// State machine per sensor: IDLE → TRIG_HIGH → WAIT_ECHO → MEASURING
enum SonicState { SONIC_IDLE, SONIC_TRIG, SONIC_WAIT, SONIC_MEASURE };

struct UltrasonicSensor {
  uint8_t trig_pin;
  uint8_t echo_pin;
  SonicState state;
  uint32_t trig_start_us;
  uint32_t echo_start_us;
  float distance_m;           // last valid reading (meters)
};

UltrasonicSensor sonic[3];    // 0=front, 1=left, 2=right
uint8_t sonic_active = 0;     // which sensor is currently cycling

// --- Motor command ---
int cmd_left_speed  = 0;      // [-255, 255]
int cmd_right_speed = 0;
uint8_t cmd_vacuum  = 0;
int current_left    = 0;      // rate-limited actual PWM
int current_right   = 0;
const int ACCEL_STEP = 10;    // max PWM change per control cycle
const int DEADBAND   = 30;    // minimum PWM to overcome static friction

// --- Communication ---
uint32_t last_cmd_time = 0;
bool pc_connected      = false;
bool first_cmd         = false;

// --- Timestamps ---
uint32_t t_sensor  = 0;
uint32_t t_comm    = 0;
uint32_t t_control = 0;
uint32_t t_oled    = 0;
uint32_t t_sonic_cycle = 0;   // inter-sensor delay

// ===========================================================================
// INTERRUPT SERVICE ROUTINES — Encoders
// ===========================================================================

// Left encoder — direction inferred from B channel
void IRAM_ATTR isr_enc_left_a() {
  int b = digitalRead(ENC_L_B);
  if (b == HIGH) enc_left++;
  else            enc_left--;
}

// Right encoder
void IRAM_ATTR isr_enc_right_a() {
  int b = digitalRead(ENC_R_B);
  if (b == HIGH) enc_right++;
  else            enc_right--;
}

// ===========================================================================
// MPU6050 — Low-level I2C helpers
// ===========================================================================

void mpu_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpu_read_raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  if (n < 14) return false;
  ax_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  ay_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  az_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // temp
  gx_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gy_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gz_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  return true;
}

void mpu_calibrate_gyro(int samples = 200) {
  // Collect gyro bias while robot is stationary
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    if (mpu_read_raw()) sum += gz_raw;
    delay(5);
  }
  gyro_z_bias = (float)sum / samples;
  Serial.printf("[MPU] Gyro Z bias = %.2f LSB\n", gyro_z_bias);
}

// ===========================================================================
// IMU — Integrate gyro Z for heading (radians)
// ===========================================================================

void imu_update() {
  if (!mpu_read_raw()) return;
  uint32_t now_us = micros();
  float dt = (now_us - imu_last_us) * 1e-6f;
  imu_last_us = now_us;
  if (dt <= 0.0f || dt > 0.5f) return;  // skip on first call or stale

  float gz_dps = (gz_raw - gyro_z_bias) / GYRO_SCALE;
  float gz_rad = gz_dps * DEG_TO_RAD;
  imu_theta += gz_rad * dt;
  // Keep in [-pi, pi]
  while (imu_theta >  M_PI) imu_theta -= 2.0f * M_PI;
  while (imu_theta < -M_PI) imu_theta += 2.0f * M_PI;
}

// ===========================================================================
// ODOMETRY — Differential drive pose integration
// ===========================================================================

void odometry_update() {
  // Snapshot encoder counts atomically
  noInterrupts();
  long el = enc_left;
  long er = enc_right;
  interrupts();

  long dl_ticks = el - prev_enc_left;
  long dr_ticks = er - prev_enc_right;
  prev_enc_left  = el;
  prev_enc_right = er;

  float dl = dl_ticks * DIST_PER_TICK;
  float dr = dr_ticks * DIST_PER_TICK;
  float dc = (dl + dr) * 0.5f;               // arc center
  float dtheta_odom = (dr - dl) / WHEEL_BASE;

  // Complementary filter: blend odometry heading with IMU heading
  float theta_odom_new = pose_theta + dtheta_odom;
  // Normalize imu_theta to match odometry reference
  float theta_fused = ALPHA * theta_odom_new + (1.0f - ALPHA) * imu_theta;

  // Integrate position using midpoint heading
  float heading_mid = pose_theta + dtheta_odom * 0.5f;
  pose_x += dc * cosf(heading_mid);
  pose_y += dc * sinf(heading_mid);
  pose_theta = theta_fused;

  // Wrap theta
  while (pose_theta >  M_PI) pose_theta -= 2.0f * M_PI;
  while (pose_theta < -M_PI) pose_theta += 2.0f * M_PI;
}

// ===========================================================================
// ULTRASONIC — Non-blocking state machine
// ===========================================================================

// Advance one step of the active sensor's state machine
void sonic_update() {
  uint32_t now_us = micros();
  UltrasonicSensor &s = sonic[sonic_active];

  switch (s.state) {

    case SONIC_IDLE:
      // Pull trig LOW to ensure clean start
      digitalWrite(s.trig_pin, LOW);
      s.state = SONIC_TRIG;
      s.trig_start_us = now_us;
      break;

    case SONIC_TRIG:
      // Hold trig LOW for 2 µs, then HIGH for 10 µs
      if (now_us - s.trig_start_us >= 2) {
        digitalWrite(s.trig_pin, HIGH);
        s.trig_start_us = now_us;
        s.state = SONIC_WAIT;
      }
      break;

    case SONIC_WAIT:
      // Wait 10 µs trig pulse, then drop LOW and start listening
      if (now_us - s.trig_start_us >= 10) {
        digitalWrite(s.trig_pin, LOW);
        s.echo_start_us = now_us;
        s.state = SONIC_MEASURE;
      }
      break;

    case SONIC_MEASURE:
      {
        int echo_val = digitalRead(s.echo_pin);
        uint32_t elapsed = now_us - s.echo_start_us;

        if (elapsed > (uint32_t)SONIC_TIMEOUT_US) {
          // No echo — treat as max range (open space, or out of range)
          s.distance_m = SONIC_MAX_RANGE;
          s.state = SONIC_IDLE;
          // Move to next sensor after short delay
          sonic_active = (sonic_active + 1) % 3;
          t_sonic_cycle = millis();
          return;
        }

        // Echo started rising — record start; falling — compute
        // We poll: when echo goes HIGH we track it, when LOW compute
        // Simple polling approach (acceptable at 20 Hz sensor rate):
        // Use pulseIn with very short timeout non-blocking approximation:
        // Since we call this every loop iteration (µs scale), it's fine.

        // Detect rising edge
        static uint32_t echo_rise_us[3] = {0,0,0};
        static bool echo_high[3] = {false,false,false};

        if (!echo_high[sonic_active] && echo_val == HIGH) {
          echo_high[sonic_active] = true;
          echo_rise_us[sonic_active] = now_us;
        } else if (echo_high[sonic_active] && echo_val == LOW) {
          uint32_t pulse_us = now_us - echo_rise_us[sonic_active];
          float dist = (pulse_us * 1e-6f * 340.0f) * 0.5f;
          if (dist > 0.02f && dist <= SONIC_MAX_RANGE) {
            s.distance_m = dist;
          }
          // else keep previous valid reading
          echo_high[sonic_active] = false;
          s.state = SONIC_IDLE;
          sonic_active = (sonic_active + 1) % 3;
          t_sonic_cycle = millis();
        }
      }
      break;
  }
}

// ===========================================================================
// MOTOR CONTROL — L298N with rate limiting
// ===========================================================================

// Apply direction + PWM to one motor channel
void motor_set(int in1_pin, int in2_pin, int pwm_ch, int speed) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);

  // Apply deadband: below threshold, don't move
  if (pwm < DEADBAND) pwm = 0;

  if (speed > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    // Brake: both HIGH for L298N
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
  }
  ledcWrite(pwm_ch, pwm);
}

// Rate-limit speed changes to avoid current spikes
int ramp(int current, int target, int step) {
  if (target > current) return min(current + step, target);
  if (target < current) return max(current - step, target);
  return current;
}

void motor_control_update() {
  // Ramp toward commanded speed
  current_left  = ramp(current_left,  cmd_left_speed,  ACCEL_STEP);
  current_right = ramp(current_right, cmd_right_speed, ACCEL_STEP);
  motor_set(MOTOR_L_IN1, MOTOR_L_IN2, PWM_CH_LEFT,  current_left);
  motor_set(MOTOR_R_IN1, MOTOR_R_IN2, PWM_CH_RIGHT, current_right);
}

void motor_stop() {
  cmd_left_speed  = 0;
  cmd_right_speed = 0;
  current_left    = 0;
  current_right   = 0;
  motor_set(MOTOR_L_IN1, MOTOR_L_IN2, PWM_CH_LEFT,  0);
  motor_set(MOTOR_R_IN1, MOTOR_R_IN2, PWM_CH_RIGHT, 0);
}

// ===========================================================================
// COMMUNICATION — JSON over UDP
// ===========================================================================

// Build and send telemetry packet to PC
void comm_send() {
  StaticJsonDocument<256> doc;
  doc["x"]         = pose_x;
  doc["y"]         = pose_y;
  doc["theta"]     = pose_theta;
  JsonArray dist   = doc.createNestedArray("distances");
  dist.add(sonic[0].distance_m);  // front
  dist.add(sonic[1].distance_m);  // left
  dist.add(sonic[2].distance_m);  // right
  doc["timestamp"] = millis();

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  udp.beginPacket(PC_IP, PC_UDP_PORT);
  udp.write((uint8_t*)buf, n);
  udp.endPacket();
}

// Parse incoming command packet from PC
void comm_receive() {
  int pkt_size = udp.parsePacket();
  if (pkt_size <= 0) return;

  char buf[128];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, buf);
  if (err) return;

  cmd_left_speed  = constrain((int)doc["left_speed"],  -255, 255);
  cmd_right_speed = constrain((int)doc["right_speed"], -255, 255);
  cmd_vacuum      = doc["vacuum"] ? 1 : 0;

  uint32_t now = millis();
  if (!first_cmd) {
    // Short beep on first valid command
    tone(BUZZER_PIN, 2000, 80);
    first_cmd = true;
  }
  last_cmd_time = now;
  pc_connected  = true;
}

// ===========================================================================
// OLED DISPLAY
// ===========================================================================

void oled_update() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);

  if (WiFi.status() == WL_CONNECTED) {
    oled.print("WiFi: OK  ");
    oled.println(WiFi.localIP());
  } else {
    oled.println("WiFi: Connecting...");
  }

  oled.printf("PC: %s\n", pc_connected ? "Connected" : "Waiting");
  oled.printf("X:%.2f Y:%.2f\n", pose_x, pose_y);
  oled.printf("Th:%.2f rad\n",    pose_theta);
  oled.printf("F:%.2f L:%.2f R:%.2f\n",
              sonic[0].distance_m,
              sonic[1].distance_m,
              sonic[2].distance_m);
  oled.display();
}

// ===========================================================================
// BUZZER HELPERS
// ===========================================================================

void beep(int freq_hz, int dur_ms) {
  tone(BUZZER_PIN, freq_hz, dur_ms);
}

void beep_error() {
  // Double low beep
  tone(BUZZER_PIN, 400, 150);
  delay(200);
  tone(BUZZER_PIN, 400, 150);
}

// ===========================================================================
// WIFI SETUP (blocking — done once in setup())
// ===========================================================================

void wifi_connect() {
  Serial.print("[WiFi] Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    beep(3000, 100);
  } else {
    Serial.println("\n[WiFi] Failed — operating offline");
    beep_error();
  }
}

// ===========================================================================
// SETUP
// ===========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("[VacBot] Booting...");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // MPU6050 init: wake up, set 250 dps gyro range
  mpu_write_reg(0x6B, 0x00);   // PWR_MGMT_1: wake
  mpu_write_reg(0x1B, 0x00);   // GYRO_CONFIG: ±250 dps
  mpu_write_reg(0x1C, 0x00);   // ACCEL_CONFIG: ±2g
  delay(100);
  mpu_calibrate_gyro(100);
  imu_last_us = micros();

  // OLED
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Init failed");
  }
  oled.clearDisplay();
  oled.display();

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  beep(1000, 200);

  // Ultrasonics
  uint8_t trig_pins[3] = {TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT};
  uint8_t echo_pins[3] = {ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT};
  for (int i = 0; i < 3; i++) {
    sonic[i] = {trig_pins[i], echo_pins[i], SONIC_IDLE, 0, 0, SONIC_MAX_RANGE};
    pinMode(trig_pins[i], OUTPUT);
    pinMode(echo_pins[i], INPUT);
    digitalWrite(trig_pins[i], LOW);
  }

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_left_a,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_right_a, CHANGE);

  // Motors
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM, PWM_CH_LEFT);
  ledcAttachPin(MOTOR_R_PWM, PWM_CH_RIGHT);
  motor_stop();

  // WiFi + UDP
  wifi_connect();
  udp.begin(ROBOT_UDP_PORT);

  uint32_t now = millis();
  t_sensor  = now;
  t_comm    = now;
  t_control = now;
  t_oled    = now;
  t_sonic_cycle = now;

  Serial.println("[VacBot] Ready.");
}

// ===========================================================================
// MAIN LOOP — Non-blocking scheduler
// ===========================================================================

void loop() {
  uint32_t now = millis();

  // --- Ultrasonic state machine (run every loop iteration) ---
  sonic_update();

  // --- Sensor loop @ ~20 Hz ---
  if (now - t_sensor >= INTERVAL_SENSOR) {
    t_sensor = now;
    imu_update();
    odometry_update();
  }

  // --- Communication loop @ ~20 Hz ---
  if (now - t_comm >= INTERVAL_COMM) {
    t_comm = now;
    comm_receive();
    if (WiFi.status() == WL_CONNECTED) {
      comm_send();
    }
    // Check command timeout → error beep
    if (first_cmd && (now - last_cmd_time) > CMD_TIMEOUT_MS) {
      pc_connected = false;
      motor_stop();
      beep_error();
    }
  }

  // --- Motor control loop @ ~50 Hz ---
  if (now - t_control >= INTERVAL_CONTROL) {
    t_control = now;
    if (pc_connected) {
      motor_control_update();
    } else {
      motor_stop();
    }
  }

  // --- OLED update @ 2 Hz ---
  if (now - t_oled >= INTERVAL_OLED) {
    t_oled = now;
    oled_update();
  }
}
