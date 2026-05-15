/*
 * VacBot SLAM Navigator - ESP32 Calibration Firmware
 *
 * Standalone utility for bench testing every subsystem before running main.ino.
 * Open Serial Monitor at 115200 baud (line ending: Newline).
 *
 * Commands (one-key, then [Enter] when arguments are required):
 *
 *   DISPLAY MODE
 *     m         cycle display: all -> ultrasonic -> encoders -> IMU -> all
 *     a u e i   jump directly to all / ultrasonic / encoders / IMU
 *
 *   MOTOR JOG (uses current pwm_step)
 *     f         both motors forward
 *     b         both motors back
 *     l         rotate left  (left -, right +)
 *     r         rotate right (left +, right -)
 *     x         stop motors immediately
 *     +/-       increase / decrease pwm_step by 10
 *
 *   UTILITIES
 *     c         calibrate gyro Z offset (hold robot still ~0.6 s)
 *     z         zero encoder counters
 *     p         print current parameters
 *     h         help
 *
 *   PARAMETER SET  (type key, then float value, then Enter)
 *     1 <val>   set wheel_radius   (m)
 *     2 <val>   set wheel_base     (m)
 *     3 <val>   set ticks_per_rev  (integer)
 *
 * Notes:
 *   - This file is independent of main.ino. It uses blocking pulseIn for
 *     ultrasonic since real-time isn't required during calibration.
 *   - Copy any tuned values back into main.ino constants (WHEEL_RADIUS,
 *     WHEEL_BASE, TICKS_PER_REV) before flashing main firmware.
 */

#include <Wire.h>
#include <math.h>

static const uint8_t PIN_SDA = 21, PIN_SCL = 22;
static const uint8_t PIN_TRIG_F = 19, PIN_ECHO_F = 18;
static const uint8_t PIN_TRIG_L = 25, PIN_ECHO_L = 26;
static const uint8_t PIN_TRIG_R = 27, PIN_ECHO_R = 33;
static const uint8_t PIN_ENC_R  = 16, PIN_ENC_L  = 17;
static const uint8_t PIN_ENA = 32, PIN_IN1 = 14, PIN_IN2 = 13;  // right
static const uint8_t PIN_IN3 = 2,  PIN_IN4 = 15, PIN_ENB = 4;   // left
static const uint8_t PIN_BUZZER = 23;

static const int LEDC_CH_A = 0, LEDC_CH_B = 1;
static const int PWM_FREQ  = 20000;
static const int PWM_RES   = 8;

// Adjustable robot parameters
static float wheel_radius   = 0.03f;
static float wheel_base     = 0.14f;
static int   ticks_per_rev  = 360;

// Encoder counters
static volatile long enc_l_count = 0;
static volatile long enc_r_count = 0;
static int last_l_dir = 0, last_r_dir = 0;

// MPU
static const uint8_t MPU_ADDR = 0x68;
static float gyro_offset_z = 0.0f;

// Mode and pacing
static int  pwm_step       = 150;
static char display_mode   = 'a';
static uint32_t last_print = 0;

// ---------- ISRs ----------
void IRAM_ATTR isr_enc_l() { if (last_l_dir >= 0) enc_l_count++; else enc_l_count--; }
void IRAM_ATTR isr_enc_r() { if (last_r_dir >= 0) enc_r_count++; else enc_r_count--; }

// ---------- MPU helpers ----------
bool mpu_w(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool mpu_r(uint8_t r, uint8_t *b, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
bool mpu_begin() {
  if (!mpu_w(0x6B, 0x00)) return false;
  delay(50);
  mpu_w(0x1B, 0x00);
  mpu_w(0x1C, 0x00);
  return true;
}
float mpu_gz_dps() {
  uint8_t b[2];
  if (!mpu_r(0x47, b, 2)) return NAN;
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  return (float)raw / 131.0f;
}
void mpu_accel(float *ax, float *ay, float *az) {
  uint8_t b[6];
  if (!mpu_r(0x3B, b, 6)) { *ax = *ay = *az = NAN; return; }
  int16_t rx = (int16_t)((b[0]<<8)|b[1]);
  int16_t ry = (int16_t)((b[2]<<8)|b[3]);
  int16_t rz = (int16_t)((b[4]<<8)|b[5]);
  *ax = rx / 16384.0f;
  *ay = ry / 16384.0f;
  *az = rz / 16384.0f;
}
void mpu_calibrate(uint16_t n) {
  double sum = 0; uint16_t got = 0;
  for (uint16_t i = 0; i < n; i++) {
    float g = mpu_gz_dps();
    if (!isnan(g)) { sum += g; got++; }
    delay(3);
  }
  gyro_offset_z = (got > 0) ? (float)(sum / got) : 0.0f;
  Serial.print(F("Gyro Z offset (deg/s) = ")); Serial.println(gyro_offset_z, 4);
}

// ---------- Ultrasonic (blocking pulseIn is fine for calibration) ----------
float read_us(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  uint32_t dt = pulseIn(echo, HIGH, 25000UL);
  if (dt == 0) return -1.0f;
  return (dt * 0.000343f) * 0.5f;
}

// ---------- Motors ----------
void apply_l(int p) {
  if (p >  255) p =  255; if (p < -255) p = -255;
  if (p > 0) last_l_dir = 1; else if (p < 0) last_l_dir = -1;
  digitalWrite(PIN_IN3, p >= 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, p >= 0 ? LOW  : HIGH);
  ledcWrite(LEDC_CH_B, abs(p));
}
void apply_r(int p) {
  if (p >  255) p =  255; if (p < -255) p = -255;
  if (p > 0) last_r_dir = 1; else if (p < 0) last_r_dir = -1;
  digitalWrite(PIN_IN1, p >= 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, p >= 0 ? LOW  : HIGH);
  ledcWrite(LEDC_CH_A, abs(p));
}
void stop_all() { apply_l(0); apply_r(0); }

// ---------- UI ----------
void help() {
  Serial.println();
  Serial.println(F("============ VacBot Calibration ============"));
  Serial.println(F("Display : m / a / u / e / i"));
  Serial.println(F("Motor   : f b l r x   (+/-) adjust pwm_step"));
  Serial.println(F("Util    : c=cal gyro  z=zero enc  p=print  h=help"));
  Serial.println(F("Params  : 1 <val>  wheel_radius (m)"));
  Serial.println(F("          2 <val>  wheel_base   (m)"));
  Serial.println(F("          3 <val>  ticks_per_rev"));
  Serial.println(F("============================================"));
}
void print_params() {
  Serial.println(F("--- Parameters ---"));
  Serial.print  (F("  wheel_radius  : ")); Serial.println(wheel_radius, 4);
  Serial.print  (F("  wheel_base    : ")); Serial.println(wheel_base,   4);
  Serial.print  (F("  ticks_per_rev : ")); Serial.println(ticks_per_rev);
  Serial.print  (F("  pwm_step      : ")); Serial.println(pwm_step);
  Serial.print  (F("  gyro_offset_z : ")); Serial.println(gyro_offset_z, 4);
}

float read_float_blocking() {
  while (Serial.available() == 0) delay(1);
  return Serial.parseFloat();
}

void handle_cmd(char c) {
  switch (c) {
    case 'h': help(); break;
    case 'a': case 'u': case 'e': case 'i':
      display_mode = c;
      Serial.print(F("display=")); Serial.println(c);
      break;
    case 'm':
      display_mode = (display_mode == 'a') ? 'u'
                   : (display_mode == 'u') ? 'e'
                   : (display_mode == 'e') ? 'i' : 'a';
      Serial.print(F("display=")); Serial.println(display_mode);
      break;
    case 'f': apply_l( pwm_step); apply_r( pwm_step); break;
    case 'b': apply_l(-pwm_step); apply_r(-pwm_step); break;
    case 'l': apply_l(-pwm_step); apply_r( pwm_step); break;
    case 'r': apply_l( pwm_step); apply_r(-pwm_step); break;
    case 'x': stop_all(); break;
    case '+': pwm_step = min(255, pwm_step + 10);
              Serial.print(F("pwm_step=")); Serial.println(pwm_step); break;
    case '-': pwm_step = max(  0, pwm_step - 10);
              Serial.print(F("pwm_step=")); Serial.println(pwm_step); break;
    case 'c': Serial.println(F("Calibrating gyro - keep still..."));
              mpu_calibrate(200); break;
    case 'z': noInterrupts(); enc_l_count = 0; enc_r_count = 0; interrupts();
              Serial.println(F("Encoders zeroed")); break;
    case 'p': print_params(); break;
    case '1': { float v = read_float_blocking(); wheel_radius = v;
                Serial.print(F("wheel_radius=")); Serial.println(v, 4); break; }
    case '2': { float v = read_float_blocking(); wheel_base = v;
                Serial.print(F("wheel_base="));   Serial.println(v, 4); break; }
    case '3': { float v = read_float_blocking(); ticks_per_rev = (int)v;
                Serial.print(F("ticks_per_rev=")); Serial.println((int)v); break; }
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  pinMode(PIN_TRIG_F, OUTPUT); pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_TRIG_L, OUTPUT); pinMode(PIN_ECHO_L, INPUT);
  pinMode(PIN_TRIG_R, OUTPUT); pinMode(PIN_ECHO_R, INPUT);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isr_enc_r, RISING);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  ledcSetup(LEDC_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(LEDC_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENA, LEDC_CH_A);
  ledcAttachPin(PIN_ENB, LEDC_CH_B);
  stop_all();

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  if (!mpu_begin()) Serial.println(F("[WARN] MPU6050 not responding"));
  help();
  print_params();
}

void loop() {
  // Read serial commands (non-blocking)
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0)                       continue;
    if (c == '\r' || c == '\n' || c == ' ') continue;
    handle_cmd((char)c);
  }

  uint32_t now = millis();
  if (now - last_print >= 200) {
    last_print = now;
    if (display_mode == 'a' || display_mode == 'u') {
      float df = read_us(PIN_TRIG_F, PIN_ECHO_F);
      float dl = read_us(PIN_TRIG_L, PIN_ECHO_L);
      float dr = read_us(PIN_TRIG_R, PIN_ECHO_R);
      Serial.print(F("US  F=")); Serial.print(df, 3);
      Serial.print(F("  L=")); Serial.print(dl, 3);
      Serial.print(F("  R=")); Serial.println(dr, 3);
    }
    if (display_mode == 'a' || display_mode == 'e') {
      noInterrupts();
      long el = enc_l_count;
      long er = enc_r_count;
      interrupts();
      Serial.print(F("ENC L=")); Serial.print(el);
      Serial.print(F("  R="));   Serial.println(er);
    }
    if (display_mode == 'a' || display_mode == 'i') {
      float gz_raw = mpu_gz_dps();
      float gz_cal = gz_raw - gyro_offset_z;
      float ax, ay, az; mpu_accel(&ax, &ay, &az);
      Serial.print(F("IMU gz=")); Serial.print(gz_cal, 2); Serial.print(F(" dps  acc=("));
      Serial.print(ax, 2); Serial.print(F(",")); Serial.print(ay, 2); Serial.print(F(","));
      Serial.print(az, 2); Serial.println(F(") g"));
    }
  }
}
