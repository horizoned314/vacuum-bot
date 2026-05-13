// =============================================================================
// VacBot SLAM Navigator — ESP32 Calibration Firmware
// =============================================================================
// Use this sketch to verify and tune hardware before deploying main.ino.
//
// Serial commands (send via Serial Monitor @ 115200 baud):
//   u   — continuous ultrasonic readings
//   e   — live encoder counts
//   m   — manual motor control (prompts for L/R speed)
//   g   — raw MPU6050 output
//   c   — calibrate gyro bias (keep robot still)
//   r   — reset encoder counts
//   s   — stop motors
//   h   — help
// =============================================================================

#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------------------------
// ADJUSTABLE CONSTANTS — tune these for your specific hardware
// ---------------------------------------------------------------------------
float WHEEL_RADIUS_M   = 0.033f;   // meters
float WHEEL_BASE_M     = 0.145f;   // meters
float TICKS_PER_REV    = 20.0f;    // quadrature ticks per full revolution
float GYRO_SCALE_LSB   = 131.0f;   // for ±250 dps range

// ---------------------------------------------------------------------------
// PIN DEFINITIONS (must match main.ino)
// ---------------------------------------------------------------------------
#define TRIG_FRONT  12
#define ECHO_FRONT  14
#define TRIG_LEFT   27
#define ECHO_LEFT   26
#define TRIG_RIGHT  25
#define ECHO_RIGHT  33

#define MOTOR_L_IN1  32
#define MOTOR_L_IN2  15
#define MOTOR_L_PWM  2
#define MOTOR_R_IN1  4
#define MOTOR_R_IN2  16
#define MOTOR_R_PWM  17

#define ENC_L_A  18
#define ENC_L_B  19
#define ENC_R_A  5
#define ENC_R_B  23

#define MPU_ADDR  0x68
#define SDA_PIN   21
#define SCL_PIN   22

#define BUZZER_PIN  13

#define PWM_FREQ        1000
#define PWM_RESOLUTION  8
#define PWM_CH_LEFT     0
#define PWM_CH_RIGHT    1

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------
volatile long enc_left  = 0;
volatile long enc_right = 0;

int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
float gyro_z_bias = 0.0f;

char mode = 'h';
bool mode_active = false;

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------
void IRAM_ATTR isr_enc_left_a() {
  enc_left += (digitalRead(ENC_L_B) == HIGH) ? 1 : -1;
}
void IRAM_ATTR isr_enc_right_a() {
  enc_right += (digitalRead(ENC_R_B) == HIGH) ? 1 : -1;
}

// ---------------------------------------------------------------------------
// MPU6050 helpers
// ---------------------------------------------------------------------------
void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpu_read() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) < 14) return false;
  ax_raw = (int16_t)(Wire.read()<<8|Wire.read());
  ay_raw = (int16_t)(Wire.read()<<8|Wire.read());
  az_raw = (int16_t)(Wire.read()<<8|Wire.read());
  Wire.read(); Wire.read(); // temp
  gx_raw = (int16_t)(Wire.read()<<8|Wire.read());
  gy_raw = (int16_t)(Wire.read()<<8|Wire.read());
  gz_raw = (int16_t)(Wire.read()<<8|Wire.read());
  return true;
}

void calibrate_gyro() {
  Serial.println("[CAL] Keep robot STILL — calibrating gyro bias over 3 s...");
  long sum = 0;
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (mpu_read()) { sum += gz_raw; n++; }
    delay(10);
  }
  gyro_z_bias = (float)sum / n;
  Serial.printf("[CAL] Gyro Z bias: %.2f LSB (= %.4f deg/s)\n",
                gyro_z_bias, gyro_z_bias / GYRO_SCALE_LSB);
}

// ---------------------------------------------------------------------------
// Ultrasonic — blocking read (OK for calibration only)
// ---------------------------------------------------------------------------
float sonic_read_cm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 25000);  // 25 ms timeout
  if (duration == 0) return -1.0f;
  float dist_m = duration * 1e-6f * 340.0f * 0.5f;
  return (dist_m <= 2.5f) ? dist_m * 100.0f : -1.0f;  // cm or -1
}

// ---------------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------------
void motor_set_raw(int left_pwm, int right_pwm) {
  left_pwm  = constrain(left_pwm,  -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  // Left
  if (left_pwm >= 0) { digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW); }
  else               { digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH); }
  ledcWrite(PWM_CH_LEFT, abs(left_pwm));

  // Right
  if (right_pwm >= 0) { digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW); }
  else                { digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH); }
  ledcWrite(PWM_CH_RIGHT, abs(right_pwm));
}

void motors_stop() {
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, HIGH);
  ledcWrite(PWM_CH_LEFT, 0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

// ---------------------------------------------------------------------------
// Print menu
// ---------------------------------------------------------------------------
void print_menu() {
  Serial.println("\n===== VacBot Calibration =====");
  Serial.printf("  WHEEL_RADIUS = %.4f m\n", WHEEL_RADIUS_M);
  Serial.printf("  WHEEL_BASE   = %.4f m\n", WHEEL_BASE_M);
  Serial.printf("  TICKS/REV    = %.1f\n",   TICKS_PER_REV);
  Serial.printf("  GYRO BIAS    = %.2f LSB\n", gyro_z_bias);
  Serial.println("Commands:");
  Serial.println("  u — ultrasonic test (continuous, ESC to stop)");
  Serial.println("  e — encoder live counts");
  Serial.println("  m — manual motor control");
  Serial.println("  g — raw MPU6050 output");
  Serial.println("  c — calibrate gyro bias");
  Serial.println("  r — reset encoders");
  Serial.println("  s — stop motors");
  Serial.println("  h — this menu");
  Serial.println("  W/X/A/D — forward/back/left/right at PWM 120");
  Serial.println("==============================\n");
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[CAL] Starting calibration firmware...");

  Wire.begin(SDA_PIN, SCL_PIN);

  // MPU6050 wake
  mpu_write(0x6B, 0x00);
  mpu_write(0x1B, 0x00);  // ±250 dps
  mpu_write(0x1C, 0x00);  // ±2g
  delay(100);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  tone(BUZZER_PIN, 1500, 150);

  // Ultrasonics
  uint8_t trigs[] = {TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT};
  uint8_t echos[] = {ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT};
  for (int i = 0; i < 3; i++) {
    pinMode(trigs[i], OUTPUT);
    pinMode(echos[i], INPUT);
    digitalWrite(trigs[i], LOW);
  }

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_right_a, CHANGE);

  // Motors
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM, PWM_CH_LEFT);
  ledcAttachPin(MOTOR_R_PWM, PWM_CH_RIGHT);
  motors_stop();

  print_menu();
}

// ===========================================================================
// LOOP
// ===========================================================================
void loop() {
  if (!Serial.available()) {
    // Continuous modes
    static uint32_t t_last = 0;
    uint32_t now = millis();

    if (mode == 'u' && now - t_last >= 200) {
      t_last = now;
      float f = sonic_read_cm(TRIG_FRONT, ECHO_FRONT);
      float l = sonic_read_cm(TRIG_LEFT,  ECHO_LEFT);
      float r = sonic_read_cm(TRIG_RIGHT, ECHO_RIGHT);
      Serial.printf("Sonic (cm)  Front: %6.1f  Left: %6.1f  Right: %6.1f\n",
                    f, l, r);
    }
    if (mode == 'e' && now - t_last >= 100) {
      t_last = now;
      noInterrupts(); long el=enc_left, er=enc_right; interrupts();
      float dist_l = el * (2.0f * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
      float dist_r = er * (2.0f * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
      Serial.printf("Enc L: %6ld (%7.3f m)  R: %6ld (%7.3f m)\n",
                    el, dist_l, er, dist_r);
    }
    if (mode == 'g' && now - t_last >= 100) {
      t_last = now;
      if (mpu_read()) {
        float gz_dps = (gz_raw - gyro_z_bias) / GYRO_SCALE_LSB;
        Serial.printf("Accel: %6d %6d %6d  Gyro: %6d %6d %6d  GzDPS: %7.3f\n",
                      ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, gz_dps);
      }
    }
    return;
  }

  char ch = (char)Serial.read();
  Serial.println(ch);  // echo

  switch (ch) {
    case 'h': print_menu(); break;

    case 'u':
      Serial.println("[SONIC] Continuous ultrasonic. Send any key to stop.");
      mode = 'u';
      break;

    case 'e':
      Serial.println("[ENC] Continuous encoder. Send any key to stop.");
      mode = 'e';
      break;

    case 'g':
      Serial.println("[IMU] Continuous MPU6050. Send any key to stop.");
      mode = 'g';
      break;

    case 'r':
      noInterrupts(); enc_left = 0; enc_right = 0; interrupts();
      Serial.println("[ENC] Reset to 0.");
      break;

    case 'c':
      mode = '0';
      calibrate_gyro();
      break;

    case 's':
      mode = '0';
      motors_stop();
      Serial.println("[MOTOR] Stopped.");
      break;

    case 'm': {
      // Manual motor: user types "left right" e.g. "120 -120"
      Serial.println("[MOTOR] Enter: left_pwm right_pwm (e.g. 120 -120)");
      // Read line
      String line = "";
      uint32_t t0 = millis();
      while (millis() - t0 < 5000) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == '\n' || c == '\r') break;
          line += c;
        }
      }
      line.trim();
      int space = line.indexOf(' ');
      if (space > 0) {
        int lp = line.substring(0, space).toInt();
        int rp = line.substring(space+1).toInt();
        Serial.printf("[MOTOR] Setting L=%d R=%d\n", lp, rp);
        motor_set_raw(lp, rp);
      } else {
        Serial.println("[MOTOR] Bad format. Example: 120 -120");
      }
      break;
    }

    // Directional shortcuts
    case 'W': motor_set_raw( 120,  120); Serial.println("[MOTOR] Forward"); break;
    case 'X': motor_set_raw(-120, -120); Serial.println("[MOTOR] Back");    break;
    case 'A': motor_set_raw(-80,   80); Serial.println("[MOTOR] Left");    break;
    case 'D': motor_set_raw( 80,  -80); Serial.println("[MOTOR] Right");   break;

    default:
      // Any other key stops continuous mode
      if (mode != '0') {
        mode = '0';
        motors_stop();
        Serial.println("[CAL] Stopped.");
        print_menu();
      }
      break;
  }
}
