// =============================================================================
//  main.ino — VacBot Standalone Mapper
//  Board  : DOIT ESP32 DEVKIT V1
//  Core   : ESP32 Arduino Core v3.x  (ledcAttach / ledcWrite by PIN, not channel)
//  Mode   : Standalone — no WiFi, no PC required
// =============================================================================
//
//  Pin map from calibration.ino (DO NOT change — matches your wiring):
//    HC-SR04 : TRIG_F=19 ECHO_F=18  TRIG_L=25 ECHO_L=26  TRIG_R=27 ECHO_R=33
//    Encoders: ENC_L=35  ENC_R=34   (GPIO 34/35 = input-only, no pull-up)
//    L298N   : ENA=32 IN1=14 IN2=13   ENB=4  IN3=2  IN4=15
//    I2C     : SDA=21  SCL=22
//    Buzzer  : 23
//    OLED    : SSD1306 128×32  addr 0x3C
//
//  Key values from your calibration output:
//    motorBalanceFactor = 0.776   → right motor faster; right PWM scaled down
//    GZ bias ≈ 0                  → gyro already near-zero at rest (good)
//
//  What it does:
//    • Non-blocking ultrasonic via interrupt-captured echo edges
//    • MPU6050 gyro-Z integrated for heading
//    • Quadrature-less encoder ISRs (direction from motor command sign)
//    • Complementary filter odometry+IMU → pose (x, y, θ)
//    • 80×80 int8 log-odds occupancy grid  (4 m × 4 m, 5 cm/cell)
//    • Bresenham ray-cast grid update
//    • 4-state autonomous exploration FSM
//    • ASCII map dump to Serial every 5 s
//    • 128×32 OLED status
//
//  Serial commands (115 200 baud):
//    p → force map print now
//    s → toggle motors (pause/resume drive while still mapping)
//    r → reset map
//
//  Libraries required (Library Manager):
//    • Adafruit SSD1306
//    • Adafruit GFX Library
//    • MPU6050 by Electronic Cats  (or jrowberg's i2cdevlib version)
// =============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"
#include <math.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
//  OLED  (128 × 32 — matches your hardware)
// ─────────────────────────────────────────────────────────────────────────────
#define SCREEN_W 128
#define SCREEN_H  32
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oled_ok = false;

// ─────────────────────────────────────────────────────────────────────────────
//  MPU6050  (library, same as calibration.ino)
// ─────────────────────────────────────────────────────────────────────────────
MPU6050 mpu;
float gyroOffsetZ = 0.0f;   // raw LSB bias, calibrated in setup()

// ─────────────────────────────────────────────────────────────────────────────
//  PIN MAP  (from calibration.ino — verified on your board)
// ─────────────────────────────────────────────────────────────────────────────
#define TRIG_F   19
#define ECHO_F   18
#define TRIG_L   25
#define ECHO_L   26
#define TRIG_R   27
#define ECHO_R   33

// GPIO 34 & 35 are INPUT-ONLY on ESP32 — no pull-up, no OUTPUT
#define ENC_L_PIN 35
#define ENC_R_PIN 34

// L298N  (left motor = IN1/IN2/ENA, right motor = IN3/IN4/ENB)
#define IN1 14
#define IN2 13
#define ENA 32
#define IN3  2
#define IN4 15
#define ENB  4

#define PIN_SDA 21
#define PIN_SCL 22
#define BUZZER  23

// ─────────────────────────────────────────────────────────────────────────────
//  ROBOT PHYSICAL CONSTANTS
//  ⚠ Verify TICKS_PER_REV with your encoder disc (spin one wheel exactly once,
//    read Serial — use calibration.ino for this)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float WHEEL_R       = 0.033f;   // wheel radius (m)
constexpr float WHEEL_BASE    = 0.145f;   // wheel-to-wheel track (m)
constexpr float TICKS_PER_REV = 20.0f;    // encoder pulses per full wheel turn
constexpr float DIST_PER_TICK = (2.0f * (float)M_PI * WHEEL_R) / TICKS_PER_REV;

// From calibration output: motorBalanceFactor = countL / countR = 0.776
// Right motor spins faster → multiply right PWM by this to equalise straight drive
constexpr float MOTOR_BALANCE = 0.776f;

constexpr float CF_ALPHA = 0.98f;   // complementary filter: 0.98 odom, 0.02 IMU

// ─────────────────────────────────────────────────────────────────────────────
//  OCCUPANCY GRID  (80×80, 5 cm/cell = 4 m × 4 m)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int   MAP_W   = 80;
constexpr int   MAP_H   = 80;
constexpr float MAP_RES = 0.05f;
constexpr int   MAP_OX  = MAP_W / 2;   // grid column = world x=0
constexpr int   MAP_OY  = MAP_H / 2;   // grid row    = world y=0

// Log-odds stored as scaled int8 (×10).  >5 = occupied, <-5 = free, else unknown.
constexpr int8_t LO_HIT    =  7;
constexpr int8_t LO_FREE   = -4;
constexpr int8_t LO_MAX    =  40;
constexpr int8_t LO_MIN    = -20;
constexpr int8_t OCC_THRESH  =  5;
constexpr int8_t FREE_THRESH = -5;

int8_t grid[MAP_H][MAP_W];   // 6 400 B — under 2 % of 520 KB SRAM

// Beam angles relative to robot heading: front=0, left=+90°, right=-90°
constexpr float BEAM[3] = { 0.0f, (float)M_PI / 2.0f, -(float)M_PI / 2.0f };

// ─────────────────────────────────────────────────────────────────────────────
//  ULTRASONIC  (interrupt-based, non-blocking)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float    SONIC_MAX  = 2.50f;
constexpr float    SONIC_NOISE = 0.02f;
constexpr uint32_t ECHO_TIMEOUT_US = 14710UL;  // 2.5 m @ 340 m/s × 2

struct EchoSlot {
  uint8_t          trig_pin, echo_pin;
  volatile uint32_t rise_us, fall_us;
  volatile bool    have_pulse;
  bool             triggered;
  uint32_t         trig_at_us;
  float            dist_m;   // last valid reading
};
EchoSlot us[3];          // 0=Front 1=Left 2=Right
uint8_t  us_active = 0; // which sensor is currently triggered

// ─────────────────────────────────────────────────────────────────────────────
//  LOCALIZATION
// ─────────────────────────────────────────────────────────────────────────────
volatile long enc_l = 0, enc_r = 0;
long prev_enc_l = 0, prev_enc_r = 0;

// Direction sign set just before applying PWM — read by ISR
volatile int8_t dir_l = 1, dir_r = 1;

float pose_x = 0.0f, pose_y = 0.0f, pose_theta = 0.0f;
float imu_theta = 0.0f;
uint32_t loc_prev_us = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR STATE
// ─────────────────────────────────────────────────────────────────────────────
constexpr int DEADBAND  = 28;   // minimum effective PWM (from calibration)
constexpr int ACCEL     = 12;   // max PWM change per 20 ms control tick

int tgt_l = 0, tgt_r = 0;   // FSM target speeds  [-255, 255]
int cur_l = 0, cur_r = 0;   // ramped actual speeds

// ─────────────────────────────────────────────────────────────────────────────
//  EXPLORATION FSM
// ─────────────────────────────────────────────────────────────────────────────
enum State : uint8_t { ST_EXPLORE, ST_BACKUP, ST_TURN_L, ST_TURN_R, ST_PAUSED };
State    bot_state      = ST_EXPLORE;
uint32_t state_enter_ms = 0;
bool     drive_on       = true;

constexpr float THR_STOP   = 0.28f;   // (m) backup immediately
constexpr float THR_SLOW   = 0.55f;   // (m) slow & choose turn
constexpr float THR_SIDE   = 0.22f;   // (m) side triggers evasion

constexpr int SPD_FWD  = 130;
constexpr int SPD_SLOW =  75;
constexpr int SPD_TURN = 110;
constexpr int SPD_BACK = 100;

// ─────────────────────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint32_t DT_SENSE   = 50;    // 20 Hz
constexpr uint32_t DT_CTRL    = 20;    // 50 Hz
constexpr uint32_t DT_OLED    = 350;   //  3 Hz  (32 px OLED is cheap)
constexpr uint32_t DT_SONIC   = 30;    // ~33 Hz round-robin
constexpr uint32_t DT_MAP     = 5000;  // map dump period

uint32_t t_sense = 0, t_ctrl = 0, t_oled = 0, t_sonic = 0, t_map = 0;

uint32_t map_updates = 0;

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
//  ENCODER ISRs
//  GPIO 34/35: input-only, no pull-up — wired to open-collector encoder output.
//  Direction is inferred from the last motor command sign (dir_l / dir_r).
// =============================================================================
void IRAM_ATTR isr_enc_l() { enc_l += dir_l; }
void IRAM_ATTR isr_enc_r() { enc_r += dir_r; }

// =============================================================================
//  ULTRASONIC ECHO ISRs  (CHANGE on each echo pin)
// =============================================================================
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
//  ULTRASONIC — trigger + finish (round-robin, called by scheduler)
// =============================================================================

// Fire the 10 µs trigger pulse.  delayMicroseconds here is unavoidable
// per the HC-SR04 datasheet; total busy-wait = 12 µs, negligible at 240 MHz.
static void us_trigger(uint8_t idx) {
  EchoSlot &s = us[idx];
  s.have_pulse  = false;
  s.triggered   = true;
  s.trig_at_us  = micros();
  digitalWrite(s.trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trig_pin, LOW);
}

// Process result of the most recently triggered sensor, then trigger next one.
static void us_step() {
  EchoSlot &s = us[us_active];

  if (s.triggered) {
    bool timed_out = (micros() - s.trig_at_us) > ECHO_TIMEOUT_US;

    if (s.have_pulse) {
      uint32_t dt = s.fall_us - s.rise_us;
      if (dt > 0 && dt < ECHO_TIMEOUT_US) {
        float d = dt * 1e-6f * 340.0f * 0.5f;
        if (d >= 0.02f && d <= SONIC_MAX) s.dist_m = d;
      }
      s.triggered = false;
    } else if (timed_out) {
      // No echo → open space or beyond range; keep last reading (safe)
      s.triggered = false;
    } else {
      return;   // still waiting for echo — don't advance yet
    }
  }

  // Advance to next sensor and fire it
  us_active = (us_active + 1) % 3;
  us_trigger(us_active);
}

// =============================================================================
//  MPU6050  (library, matching calibration.ino)
// =============================================================================

static void calibrate_gyro(int samples = 500) {
  Serial.print(F("[MPU] Calibrating gyro — keep still..."));
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

// Read calibrated gz in rad/s — called at ~20 Hz
static float read_gz_rads() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float gz_dps = (gz - gyroOffsetZ) / 131.0f;
  return gz_dps * ((float)M_PI / 180.0f);
}

// =============================================================================
//  ODOMETRY + COMPLEMENTARY FILTER
// =============================================================================
static void localization_update() {
  uint32_t now_us = micros();
  float dt = (now_us - loc_prev_us) * 1e-6f;
  loc_prev_us = now_us;
  if (dt <= 0.0f || dt > 0.3f) return;

  // Snapshot encoder counts atomically
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

  // IMU integration
  float gz = read_gz_rads();
  imu_theta = wrap_pi(imu_theta + gz * dt);

  // Complementary filter on heading
  float th_odom = pose_theta + dth;
  float diff    = wrap_pi(imu_theta - pose_theta);
  pose_theta    = wrap_pi(pose_theta
                          + CF_ALPHA * dth
                          + (1.0f - CF_ALPHA) * diff);

  // Position integration (midpoint method)
  float mid_th = pose_theta - dth * 0.5f;
  pose_x += dc * cosf(mid_th);
  pose_y += dc * sinf(mid_th);
}

// =============================================================================
//  OCCUPANCY GRID
// =============================================================================
static bool cell_ok(int c, int r) {
  return c >= 0 && c < MAP_W && r >= 0 && r < MAP_H;
}

static void world_to_cell(float wx, float wy, int &col, int &row) {
  col = (int)roundf(MAP_OX + wx / MAP_RES);
  row = (int)roundf(MAP_OY - wy / MAP_RES);   // y-left: decreases with row
}

static void mark_free(int c, int r) {
  if (!cell_ok(c, r)) return;
  int v = (int)grid[r][c] + LO_FREE;
  grid[r][c] = (int8_t)(v < LO_MIN ? LO_MIN : v);
}

static void mark_occ(int c, int r) {
  if (!cell_ok(c, r)) return;
  int v = (int)grid[r][c] + LO_HIT;
  grid[r][c] = (int8_t)(v > LO_MAX ? LO_MAX : v);
}

// Bresenham ray: mark path free, endpoint occupied if is_hit
static void cast_ray(int c0, int r0, int c1, int r1, bool is_hit) {
  int dc = abs(c1 - c0), dr = abs(r1 - r0);
  int sc = c1 > c0 ? 1 : -1;
  int sr = r1 > r0 ? 1 : -1;
  int err = dc - dr, c = c0, r = r0;
  for (int step = 0; step < 250; step++) {
    if (c == c1 && r == r1) { if (is_hit) mark_occ(c, r); break; }
    mark_free(c, r);
    int e2 = 2 * err;
    if (e2 > -dr) { err -= dr; c += sc; }
    if (e2 <  dc) { err += dc; r += sr; }
  }
}

static void grid_update() {
  int rc, rr;
  world_to_cell(pose_x, pose_y, rc, rr);

  for (int i = 0; i < 3; i++) {
    float d = us[i].dist_m;
    if (d < 0.02f || d > SONIC_MAX) continue;

    float angle = pose_theta + BEAM[i];
    float hx = pose_x + d * cosf(angle);
    float hy = pose_y + d * sinf(angle);
    int hc, hr;
    world_to_cell(hx, hy, hc, hr);

    cast_ray(rc, rr, hc, hr, d < SONIC_MAX - SONIC_NOISE);
  }
  map_updates++;
}

static void grid_reset() {
  memset(grid, 0, sizeof(grid));
  map_updates = 0;
}

// =============================================================================
//  MOTOR CONTROL  (ESP32 v3.x LEDC API — ledcAttach / ledcWrite by PIN)
// =============================================================================

static void apply_left(int pwm) {
  pwm = constrain(pwm, -255, 255);
  dir_l = (pwm >= 0) ? 1 : -1;   // update direction for ISR
  if      (pwm > 0)  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (pwm < 0)  { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }  // coast
  ledcWrite(ENA, (pwm == 0) ? 0 : max(DEADBAND, abs(pwm)));
}

static void apply_right(int pwm) {
  pwm = constrain(pwm, -255, 255);
  // Apply balance factor: right motor is faster (0.776 < 1)
  // Scale right down so both wheels travel equal distance at same command
  int pwm_bal = (int)(pwm * MOTOR_BALANCE);
  pwm_bal = constrain(pwm_bal, -255, 255);
  dir_r = (pwm >= 0) ? 1 : -1;   // direction tracks unscaled command sign
  if      (pwm_bal > 0)  { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (pwm_bal < 0)  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else                   { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
  ledcWrite(ENB, (pwm_bal == 0) ? 0 : max(DEADBAND, abs(pwm_bal)));
}

static void motor_tick() {
  if (!drive_on) { cur_l = cur_r = 0; apply_left(0); apply_right(0); return; }
  cur_l += constrain(tgt_l - cur_l, -ACCEL, ACCEL);
  cur_r += constrain(tgt_r - cur_r, -ACCEL, ACCEL);
  apply_left(cur_l);
  apply_right(cur_r);
}

static void motors_stop_now() {
  tgt_l = tgt_r = cur_l = cur_r = 0;
  apply_left(0); apply_right(0);
}

// =============================================================================
//  EXPLORATION FSM
// =============================================================================
static const char* state_str() {
  switch (bot_state) {
    case ST_EXPLORE: return "EXPLORE";
    case ST_BACKUP:  return "BACKUP ";
    case ST_TURN_L:  return "TURN_L ";
    case ST_TURN_R:  return "TURN_R ";
    case ST_PAUSED:  return "PAUSED ";
    default:         return "?      ";
  }
}

static void go(State next, int beep_hz = 0) {
  bot_state      = next;
  state_enter_ms = millis();
  if (beep_hz) beep(beep_hz, 55);
}

static void fsm_tick() {
  float df = us[0].dist_m;
  float dl = us[1].dist_m;
  float dr = us[2].dist_m;
  uint32_t age = millis() - state_enter_ms;

  switch (bot_state) {

    case ST_EXPLORE:
      // Obstacle straight ahead — back up first
      if (df < THR_STOP) { go(ST_BACKUP, 900); break; }

      // Front partially blocked — turn toward more open side
      if (df < THR_SLOW) {
        (dl >= dr) ? go(ST_TURN_L, 1400) : go(ST_TURN_R, 1400);
        break;
      }

      // Side obstacles — lateral evasion
      if (dr < THR_SIDE) { go(ST_TURN_L, 1400); break; }
      if (dl < THR_SIDE) { go(ST_TURN_R, 1400); break; }

      // Normal forward — bias toward open side for gentle wall following
      {
        int spd = (df < THR_SLOW * 1.5f) ? SPD_SLOW : SPD_FWD;
        float bias = 0.0f;
        if (dl > 0.45f && dr < 0.30f) bias = -0.18f;   // pull left (more open)
        if (dr > 0.45f && dl < 0.30f) bias =  0.18f;   // pull right
        tgt_l = constrain((int)(spd * (1.0f + bias)), -255, 255);
        tgt_r = constrain((int)(spd * (1.0f - bias)), -255, 255);
      }
      break;

    case ST_BACKUP:
      tgt_l = tgt_r = -SPD_BACK;
      if (age > 550) { (dl >= dr) ? go(ST_TURN_L, 1400) : go(ST_TURN_R, 1400); }
      break;

    case ST_TURN_L:
      tgt_l = -SPD_TURN; tgt_r = SPD_TURN;
      if (age > 350 && df > THR_SLOW) go(ST_EXPLORE, 2100);
      if (age > 2400) go(ST_EXPLORE, 2100);   // timeout safety
      break;

    case ST_TURN_R:
      tgt_l = SPD_TURN; tgt_r = -SPD_TURN;
      if (age > 350 && df > THR_SLOW) go(ST_EXPLORE, 2100);
      if (age > 2400) go(ST_EXPLORE, 2100);
      break;

    case ST_PAUSED:
      tgt_l = tgt_r = 0;
      break;
  }
}

// =============================================================================
//  OLED  (128 × 32 — 4 lines at textSize 1)
// =============================================================================
static void oled_update() {
  if (!oled_ok) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  // Line 0: state + update count
  display.printf("%-7s upd:%-5lu\n", state_str(), map_updates);
  // Line 1: pose
  display.printf("X%5.2f Y%5.2f T%4.0f\n",
                 pose_x, pose_y, pose_theta * 180.0f / (float)M_PI);
  // Line 2: sensor distances (cm for compactness)
  display.printf("F%3.0f L%3.0f R%3.0f cm\n",
                 us[0].dist_m * 100.0f,
                 us[1].dist_m * 100.0f,
                 us[2].dist_m * 100.0f);
  // Line 3: encoder counts
  display.printf("EL:%-5ld ER:%-5ld\n", enc_l, enc_r);
  display.display();
}

// =============================================================================
//  ASCII MAP DUMP  (40×40 downsampled view of 80×80 grid)
//  '#' occupied  '.' free  ' ' unknown  'R' robot position
// =============================================================================
static void print_map() {
  int rc, rr;
  world_to_cell(pose_x, pose_y, rc, rr);

  Serial.println(F("\n+----------------------------------------+"));
  Serial.printf("| %-7s  x=%5.2f y=%5.2f th=%5.1f  |\n",
                state_str(), pose_x, pose_y,
                pose_theta * 180.0f / (float)M_PI);
  Serial.printf("| F=%.2fm L=%.2fm R=%.2fm  upd=%-6lu |\n",
                us[0].dist_m, us[1].dist_m, us[2].dist_m, map_updates);
  Serial.println(F("+----------------------------------------+"));

  // Top border
  Serial.print('+');
  for (int c = 0; c < MAP_W / 2; c++) Serial.print('-');
  Serial.println('+');

  for (int r = 0; r < MAP_H; r += 2) {
    Serial.print('|');
    for (int c = 0; c < MAP_W; c += 2) {
      if (abs(c - rc) <= 1 && abs(r - rr) <= 1) { Serial.print('R'); continue; }
      int8_t v = grid[r][c];
      if      (v >= OCC_THRESH)   Serial.print('#');
      else if (v <= FREE_THRESH)  Serial.print('.');
      else                        Serial.print(' ');
    }
    Serial.println('|');
  }

  Serial.print('+');
  for (int c = 0; c < MAP_W / 2; c++) Serial.print('-');
  Serial.println('+');
  Serial.println(F("Commands: p=map  s=motors  r=reset\n"));
}

// =============================================================================
//  SERIAL COMMANDS
// =============================================================================
static void serial_check() {
  if (!Serial.available()) return;
  char ch = (char)Serial.read();
  switch (ch) {
    case 'p': case 'P':
      print_map(); break;
    case 's': case 'S':
      drive_on = !drive_on;
      if (!drive_on) motors_stop_now();
      Serial.printf("[DRIVE] %s\n", drive_on ? "ON" : "OFF");
      beep(drive_on ? 1800 : 500, 80);
      break;
    case 'r': case 'R':
      grid_reset();
      Serial.println(F("[MAP] Reset."));
      beep(400, 200);
      break;
    default: break;
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n[VacBot] Booting..."));

  // ── I²C ──────────────────────────────────────────────────────────────────
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // ── OLED (128×32) ─────────────────────────────────────────────────────────
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

  // ── Buzzer ────────────────────────────────────────────────────────────────
  pinMode(BUZZER, OUTPUT);
  beep(1000, 150);

  // ── MPU6050 ───────────────────────────────────────────────────────────────
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("[MPU] OK"));
    calibrate_gyro(500);   // ~2 s, keep robot still
  } else {
    Serial.println(F("[WARN] MPU6050 not found — heading from encoder only."));
  }
  loc_prev_us = micros();

  // ── HC-SR04 pins + echo ISRs ──────────────────────────────────────────────
  const uint8_t TRIGS[3] = { TRIG_F, TRIG_L, TRIG_R };
  const uint8_t ECHOS[3] = { ECHO_F, ECHO_L, ECHO_R };
  for (int i = 0; i < 3; i++) {
    us[i] = { TRIGS[i], ECHOS[i], 0, 0, false, false, 0, SONIC_MAX };
    pinMode(TRIGS[i], OUTPUT);
    digitalWrite(TRIGS[i], LOW);
    pinMode(ECHOS[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ECHO_F), isr_echo_f, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), isr_echo_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), isr_echo_r, CHANGE);

  // ── Encoders (GPIO 34/35 = input-only, no PULLUP) ────────────────────────
  pinMode(ENC_L_PIN, INPUT);
  pinMode(ENC_R_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isr_enc_r, RISING);

  // ── L298N motor driver (ESP32 v3.x LEDC API) ──────────────────────────────
  //  ledcAttach(pin, freq_hz, resolution_bits) — no channel numbers in v3.x
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);   // same call as calibration.ino
  ledcAttach(ENB, 1000, 8);
  motors_stop_now();

  // ── Grid init ─────────────────────────────────────────────────────────────
  memset(grid, 0, sizeof(grid));

  // ── Start first ultrasonic measurement ───────────────────────────────────
  us_trigger(0);

  // ── Timer baselines ───────────────────────────────────────────────────────
  uint32_t now = millis();
  t_sense = t_ctrl = t_oled = t_sonic = now;
  t_map   = now + 4000;   // first dump 4 s after boot

  go(ST_EXPLORE);
  beep(2000, 80);
  Serial.println(F("[VacBot] Ready — exploring."));
  Serial.println(F("Commands: p=map  s=toggle motors  r=reset map\n"));
}

// =============================================================================
//  LOOP  — non-blocking scheduler, no delay()
// =============================================================================
void loop() {
  uint32_t now = millis();

  // ── ~33 Hz: ultrasonic round-robin ───────────────────────────────────────
  if (now - t_sonic >= DT_SONIC) {
    t_sonic = now;
    us_step();
  }

  // ── 20 Hz: IMU + odometry + grid update ──────────────────────────────────
  if (now - t_sense >= DT_SENSE) {
    t_sense = now;
    localization_update();
    grid_update();

    Serial.printf("[%7lu] %-7s x=%5.2f y=%5.2f th=%5.1f | "
                  "F=%.2f L=%.2f R=%.2f | EL=%ld ER=%ld\n",
                  now, state_str(),
                  pose_x, pose_y,
                  pose_theta * 180.0f / (float)M_PI,
                  us[0].dist_m, us[1].dist_m, us[2].dist_m,
                  enc_l, enc_r);
  }

  // ── 50 Hz: FSM + motor ramp ───────────────────────────────────────────────
  if (now - t_ctrl >= DT_CTRL) {
    t_ctrl = now;
    fsm_tick();
    motor_tick();
  }

  // ── 3 Hz: OLED ────────────────────────────────────────────────────────────
  if (now - t_oled >= DT_OLED) {
    t_oled = now;
    oled_update();
  }

  // ── 0.2 Hz: ASCII map dump ────────────────────────────────────────────────
  if (now - t_map >= DT_MAP) {
    t_map = now;
    print_map();
  }

  // ── Serial commands ───────────────────────────────────────────────────────
  serial_check();
}
