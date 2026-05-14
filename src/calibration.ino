// =============================================================================
// VacBot SLAM Navigator — ESP32 Calibration Firmware  (rev-B wiring)
// =============================================================================
// Hardware:
//   MPU6050        SDA=D21  SCL=D22
//   SSD1306 OLED   SDA=D21  SCL=D22  (shared I2C bus, addr 0x3C)
//   HC-SR04 Front  TRIG=D19 ECHO=D18
//   HC-SR04 Left   TRIG=D25 ECHO=D26
//   HC-SR04 Right  TRIG=D27 ECHO=D33
//   L298N          ENA=D32  IN1=D14  IN2=D13
//                  ENB=D4   IN3=D2   IN4=D15
//   Encoder Left   D0=D17  (single-channel, direction from motor state)
//   Encoder Right  D0=D16  (single-channel, direction from motor state)
//   Buzzer (active)VCC=D23
//
// Required libraries (install via Library Manager):
//   Adafruit SSD1306   https://github.com/adafruit/Adafruit_SSD1306
//   Adafruit GFX       https://github.com/adafruit/Adafruit-GFX-Library
//
// Serial commands @ 115200 baud:
//   u — continuous ultrasonic readings
//   e — live encoder counts
//   m — manual motor control (prompts for L/R speed)
//   g — raw MPU6050 / gyro output
//   c — calibrate gyro bias (keep robot still)
//   r — reset encoder counts
//   s — stop motors
//   h — help / show menu
//   W/X/A/D — forward / back / left / right at PWM 140
// =============================================================================

#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// ADJUSTABLE CONSTANTS — tune for your hardware
// ---------------------------------------------------------------------------
float WHEEL_RADIUS_M = 0.033f;   // metres
float WHEEL_BASE_M   = 0.145f;   // metres (distance between wheel centres)
float TICKS_PER_REV  = 20.0f;    // pulses per full wheel revolution
float GYRO_SCALE_LSB = 131.0f;   // LSB/(°/s) for ±250 dps range

// ---------------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------------
// Ultrasonic sensors
#define TRIG_FRONT  19
#define ECHO_FRONT  18
#define TRIG_LEFT   25
#define ECHO_LEFT   26
#define TRIG_RIGHT  27
#define ECHO_RIGHT  33

// L298N motor driver
//   Left  motor  → ENA (PWM), IN1, IN2
//   Right motor  → ENB (PWM), IN3, IN4
#define MOTOR_L_ENA  32   // PWM
#define MOTOR_L_IN1  14
#define MOTOR_L_IN2  13
#define MOTOR_R_ENB  4    // PWM
#define MOTOR_R_IN3  2
#define MOTOR_R_IN4  15

// Single-channel optical encoders (D0 output only)
#define ENC_LEFT   17
#define ENC_RIGHT  16

// I2C (shared by MPU6050 + OLED)
#define SDA_PIN  21
#define SCL_PIN  22

// MPU6050
#define MPU_ADDR  0x68

// SSD1306 OLED
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR     0x3C

// Active buzzer (HIGH = on)
#define BUZZER_PIN  23

// LEDC PWM channels
#define PWM_FREQ        1000
#define PWM_RESOLUTION  8        // 0–255
#define PWM_CH_LEFT     0
#define PWM_CH_RIGHT    1

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Encoders — direction tracked from last motor command
volatile long enc_left  = 0;
volatile long enc_right = 0;
volatile int8_t dir_left  = 1;   // +1 forward, -1 reverse
volatile int8_t dir_right = 1;

// MPU6050 raw values
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
float   gyro_z_bias = 0.0f;

// Current serial mode
char mode = '0';

// ---------------------------------------------------------------------------
// ISR — single-channel: use stored direction flag
// ---------------------------------------------------------------------------
void IRAM_ATTR isr_enc_left() {
  enc_left += dir_left;
}
void IRAM_ATTR isr_enc_right() {
  enc_right += dir_right;
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
  ax_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  ay_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  az_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // temperature (skip)
  gx_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gy_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gz_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  return true;
}

void calibrate_gyro() {
  Serial.println("[CAL] Keep robot STILL — sampling gyro bias for 3 s ...");
  oled_msg("Gyro cal...", "Keep still!");

  long sum = 0;
  int  n   = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (mpu_read()) { sum += gz_raw; n++; }
    delay(10);
  }
  gyro_z_bias = (n > 0) ? (float)sum / n : 0.0f;
  Serial.printf("[CAL] Gyro Z bias: %.2f LSB  (%.4f deg/s)\n",
                gyro_z_bias, gyro_z_bias / GYRO_SCALE_LSB);

  char buf[32];
  snprintf(buf, sizeof(buf), "Bias: %.2f", gyro_z_bias);
  oled_msg("Gyro done", buf);
}

// ---------------------------------------------------------------------------
// OLED helpers
// ---------------------------------------------------------------------------
void oled_msg(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 12);
  display.println(line2);
  display.display();
}

// Update OLED with sonic / encoder data (called in continuous modes)
void oled_sonic(float f, float l, float r) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.print("F:");  display.print(f, 1); display.print(" cm");
  display.setCursor(0, 12); display.print("L:");  display.print(l, 1); display.print(" cm");
  display.setCursor(0, 24); display.print("R:");  display.print(r, 1); display.print(" cm");
  display.display();
}

void oled_encoder(long el, long er, float dl, float dr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.printf("L ticks: %ld", el);
  display.setCursor(0, 12); display.printf("L dist:  %.3f m", dl);
  display.setCursor(0, 28); display.printf("R ticks: %ld", er);
  display.setCursor(0, 40); display.printf("R dist:  %.3f m", dr);
  display.display();
}

void oled_imu(float gz_dps) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.print("IMU (MPU6050)");
  display.setCursor(0, 12); display.printf("Ax: %d", ax_raw);
  display.setCursor(0, 22); display.printf("Ay: %d", ay_raw);
  display.setCursor(0, 32); display.printf("Az: %d", az_raw);
  display.setCursor(0, 44); display.printf("Gz: %.2f dps", gz_dps);
  display.display();
}

// ---------------------------------------------------------------------------
// Ultrasonic — blocking (calibration only)
// ---------------------------------------------------------------------------
float sonic_read_cm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000);  // 25 ms timeout → ~4 m max
  if (dur == 0) return -1.0f;
  float dist_m = dur * 1e-6f * 340.0f * 0.5f;
  return (dist_m <= 2.5f) ? dist_m * 100.0f : -1.0f;
}

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------
void motor_set_raw(int left_pwm, int right_pwm) {
  left_pwm  = constrain(left_pwm,  -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  // Update direction flags for encoder counting
  dir_left  = (left_pwm  >= 0) ? 1 : -1;
  dir_right = (right_pwm >= 0) ? 1 : -1;

  // Left motor
  if (left_pwm >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
  }
  ledcWrite(PWM_CH_LEFT, abs(left_pwm));

  // Right motor
  if (right_pwm >= 0) {
    digitalWrite(MOTOR_R_IN3, HIGH);
    digitalWrite(MOTOR_R_IN4, LOW);
  } else {
    digitalWrite(MOTOR_R_IN3, LOW);
    digitalWrite(MOTOR_R_IN4, HIGH);
  }
  ledcWrite(PWM_CH_RIGHT, abs(right_pwm));
}

void motors_stop() {
  // Brake: both IN pins HIGH, PWM 0
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN3, HIGH); digitalWrite(MOTOR_R_IN4, HIGH);
  ledcWrite(PWM_CH_LEFT,  0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

// ---------------------------------------------------------------------------
// Buzzer (active — just a digital pulse)
// ---------------------------------------------------------------------------
void buzzer_beep(int ms = 100) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// ---------------------------------------------------------------------------
// Serial menu
// ---------------------------------------------------------------------------
void print_menu() {
  Serial.println("\n===== VacBot Calibration (rev-B) =====");
  Serial.printf("  WHEEL_RADIUS = %.4f m\n", WHEEL_RADIUS_M);
  Serial.printf("  WHEEL_BASE   = %.4f m\n", WHEEL_BASE_M);
  Serial.printf("  TICKS/REV    = %.1f\n",   TICKS_PER_REV);
  Serial.printf("  GYRO BIAS    = %.2f LSB\n", gyro_z_bias);
  Serial.println("Commands:");
  Serial.println("  u — ultrasonic (continuous, any key stops)");
  Serial.println("  e — encoder live counts");
  Serial.println("  m — manual motor control");
  Serial.println("  g — raw MPU6050 output");
  Serial.println("  c — calibrate gyro bias (keep still)");
  Serial.println("  r — reset encoders to 0");
  Serial.println("  s — stop motors");
  Serial.println("  h — this menu");
  Serial.println("  W/X/A/D — forward/back/left/right @ PWM 140");
  Serial.println("=======================================\n");
  oled_msg("VacBot Cal", "Send 'h' help");
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[CAL] Starting calibration firmware (rev-B)...");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] init FAILED — check wiring / address");
  } else {
    display.clearDisplay();
    display.display();
    Serial.println("[OLED] OK");
  }

  // MPU6050 — wake + set ranges
  mpu_write(0x6B, 0x00);  // wake
  mpu_write(0x1B, 0x00);  // gyro ±250 dps
  mpu_write(0x1C, 0x00);  // accel ±2g
  delay(100);
  if (mpu_read()) {
    Serial.println("[IMU] MPU6050 OK");
  } else {
    Serial.println("[IMU] MPU6050 FAILED — check wiring");
  }

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer_beep(150);

  // Ultrasonic
  const uint8_t trigs[] = {TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT};
  const uint8_t echos[] = {ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT};
  for (int i = 0; i < 3; i++) {
    pinMode(trigs[i], OUTPUT);
    pinMode(echos[i], INPUT);
    digitalWrite(trigs[i], LOW);
  }

  // Encoders (single-channel D0)
  pinMode(ENC_LEFT,  INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT),  isr_enc_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), isr_enc_right, RISING);

  // Motor driver pins
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT); pinMode(MOTOR_R_IN4, OUTPUT);

  // LEDC PWM (ESP32 Arduino ≤ 2.x API)
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_ENA, PWM_CH_LEFT);
  ledcAttachPin(MOTOR_R_ENB, PWM_CH_RIGHT);
  motors_stop();

  print_menu();
}

// ===========================================================================
// LOOP
// ===========================================================================
void loop() {
  if (!Serial.available()) {
    // ---- Continuous display modes ----
    static uint32_t t_last = 0;
    uint32_t now = millis();

    if (mode == 'u' && now - t_last >= 200) {
      t_last = now;
      float f = sonic_read_cm(TRIG_FRONT, ECHO_FRONT);
      float l = sonic_read_cm(TRIG_LEFT,  ECHO_LEFT);
      float r = sonic_read_cm(TRIG_RIGHT, ECHO_RIGHT);
      Serial.printf("Sonic(cm) F:%6.1f  L:%6.1f  R:%6.1f\n", f, l, r);
      oled_sonic(f, l, r);
    }

    if (mode == 'e' && now - t_last >= 100) {
      t_last = now;
      noInterrupts();
      long el = enc_left, er = enc_right;
      interrupts();
      float dl = el * (2.0f * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
      float dr = er * (2.0f * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
      Serial.printf("Enc  L:%6ld (%7.3f m)  R:%6ld (%7.3f m)\n",
                    el, dl, er, dr);
      oled_encoder(el, er, dl, dr);
    }

    if (mode == 'g' && now - t_last >= 100) {
      t_last = now;
      if (mpu_read()) {
        float gz_dps = (gz_raw - gyro_z_bias) / GYRO_SCALE_LSB;
        Serial.printf("Accel:%6d %6d %6d  Gyro:%6d %6d %6d  Gz:%.3f dps\n",
                      ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, gz_dps);
        oled_imu(gz_dps);
      }
    }
    return;
  }

  // ---- Serial command handler ----
  char ch = (char)Serial.read();
  Serial.println(ch);  // echo back

  switch (ch) {

    case 'h':
      print_menu();
      break;

    case 'u':
      Serial.println("[SONIC] Continuous ultrasonic. Send any key to stop.");
      oled_msg("Sonic mode", "Any key stop");
      mode = 'u';
      break;

    case 'e':
      Serial.println("[ENC] Continuous encoder. Send any key to stop.");
      oled_msg("Encoder mode", "Any key stop");
      mode = 'e';
      break;

    case 'g':
      Serial.println("[IMU] Continuous MPU6050. Send any key to stop.");
      oled_msg("IMU mode", "Any key stop");
      mode = 'g';
      break;

    case 'r':
      noInterrupts();
      enc_left  = 0;
      enc_right = 0;
      interrupts();
      Serial.println("[ENC] Encoders reset to 0.");
      oled_msg("Encoders", "Reset to 0");
      break;

    case 'c':
      mode = '0';
      motors_stop();
      calibrate_gyro();
      break;

    case 's':
      mode = '0';
      motors_stop();
      Serial.println("[MOTOR] Stopped.");
      oled_msg("Motors", "Stopped");
      break;

    case 'm': {
      // Prompt for "left_pwm right_pwm"
      Serial.println("[MOTOR] Enter: left_pwm right_pwm  (e.g. 140 -140)");
      String line = "";
      uint32_t t0 = millis();
      while (millis() - t0 < 6000) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == '\n' || c == '\r') break;
          line += c;
        }
      }
      line.trim();
      int sp = line.indexOf(' ');
      if (sp > 0) {
        int lp = line.substring(0, sp).toInt();
        int rp = line.substring(sp + 1).toInt();
        Serial.printf("[MOTOR] L=%d  R=%d\n", lp, rp);
        char buf[32];
        snprintf(buf, sizeof(buf), "L=%d R=%d", lp, rp);
        oled_msg("Motor manual", buf);
        motor_set_raw(lp, rp);
      } else {
        Serial.println("[MOTOR] Bad format. Example: 140 -140");
      }
      break;
    }

    // Direction shortcuts (uppercase to avoid accidental firing)
    case 'W':
      motor_set_raw(140, 140);
      Serial.println("[MOTOR] Forward");
      oled_msg("Motor", "Forward");
      break;
    case 'X':
      motor_set_raw(-140, -140);
      Serial.println("[MOTOR] Backward");
      oled_msg("Motor", "Backward");
      break;
    case 'A':
      motor_set_raw(-100, 100);
      Serial.println("[MOTOR] Spin Left");
      oled_msg("Motor", "Spin Left");
      break;
    case 'D':
      motor_set_raw(100, -100);
      Serial.println("[MOTOR] Spin Right");
      oled_msg("Motor", "Spin Right");
      break;

    default:
      // Any unrecognised key stops continuous mode
      if (mode != '0') {
        mode = '0';
        motors_stop();
        Serial.println("[CAL] Stopped continuous mode.");
        print_menu();
      }
      break;
  }
}
