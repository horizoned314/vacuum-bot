#include <Arduino.h>
#include <Wire.h>
#include <cstring>

#include <PS4Controller.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ======================================================
// PIN CONFIG
// ======================================================

namespace Pin {
  // I2C
  constexpr uint8_t I2C_SDA = 21;
  constexpr uint8_t I2C_SCL = 22;

  // L298N - Left motor
  constexpr uint8_t LEFT_EN  = 25;
  constexpr uint8_t LEFT_IN1 = 26;
  constexpr uint8_t LEFT_IN2 = 27;

  // L298N - Right motor
  constexpr uint8_t RIGHT_EN  = 13;
  constexpr uint8_t RIGHT_IN1 = 14;
  constexpr uint8_t RIGHT_IN2 = 32;

  // Ultrasonic HC-SR04
  constexpr uint8_t TRIG_FRONT = 5;
  constexpr uint8_t ECHO_FRONT = 34;

  constexpr uint8_t TRIG_LEFT = 18;
  constexpr uint8_t ECHO_LEFT = 35;

  constexpr uint8_t TRIG_RIGHT = 19;
  constexpr uint8_t ECHO_RIGHT = 36;

  // Encoder, one channel per wheel
  constexpr uint8_t ENC_LEFT_A  = 39;
  constexpr uint8_t ENC_RIGHT_A = 23;

  // Active buzzer
  constexpr uint8_t BUZZER = 4;

}

// ======================================================
// USER TUNING
// ======================================================

// Isi jika Anda ingin ESP32 memakai MAC tertentu untuk PS4.begin("xx:xx:xx:xx:xx:xx").
// Kosongkan "" untuk memakai Bluetooth MAC ESP32 default.
const char *DS4_HOST_MAC = "";

// Aktifkan true hanya jika Anda menambahkan driver vacuum, misalnya MOSFET module.
//constexpr bool ENABLE_VACUUM_OUTPUT = false;

// OLED 0.91 inch SSD1306 128x32 I2C
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 32;
constexpr int OLED_RESET = -1;

// PWM motor
constexpr uint32_t PWM_FREQ = 20000;   // 20 kHz
constexpr uint8_t PWM_RES_BITS = 8;    // 0..255
constexpr uint8_t PWM_CH_LEFT = 0;     // dipakai di Arduino-ESP32 v2
constexpr uint8_t PWM_CH_RIGHT = 1;    // dipakai di Arduino-ESP32 v2

// Kompensasi motor gearbox kuning biasanya tidak mulai berputar di PWM kecil.
// Turunkan/naikkan sesuai motor Anda.
constexpr int MOTOR_MIN_PWM = 70;
constexpr int MOTOR_MAX_PWM = 255;

// Obstacle avoidance
constexpr uint16_t FRONT_STOP_CM = 22;
constexpr uint16_t SIDE_STOP_CM = 14;
constexpr uint16_t MANUAL_FRONT_GUARD_CM = 10;

constexpr int AUTO_FORWARD_PWM = 135;
constexpr int AUTO_STEER_FAST_PWM = 135;
constexpr int AUTO_STEER_SLOW_PWM = 80;
constexpr int AUTO_REVERSE_PWM = 120;
constexpr int AUTO_TURN_PWM = 130;

constexpr uint32_t AUTO_BACKUP_MS = 420;
constexpr uint32_t AUTO_TURN_MS = 520;

// HC-SR04 timeout.
// 25000 us kira-kira cukup untuk jarak beberapa meter.
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 25000;
constexpr uint16_t DIST_MAX_CM = 400;

// Encoder calibration.
// Sesuaikan dengan encoder/wheel Anda.
constexpr float WHEEL_DIAMETER_CM = 6.5f;
constexpr int ENCODER_TICKS_PER_REV = 20;
constexpr float CM_PER_TICK =
    (3.1415926f * WHEEL_DIAMETER_CM) / ENCODER_TICKS_PER_REV;

// ======================================================
// GLOBALS
// ======================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

bool hasDisplay = false;
bool hasMpu = false;

enum class DriveMode : uint8_t {
  AUTO,
  MANUAL
};

enum class AutoState : uint8_t {
  CRUISE,
  BACKUP,
  TURN_LEFT,
  TURN_RIGHT
};

DriveMode driveMode = DriveMode::AUTO;
AutoState autoState = AutoState::CRUISE;

uint32_t autoStateUntil = 0;

bool emergencyStop = false;
bool vacuumOn = false;
bool ps4WasConnected = false;

uint16_t distFront = DIST_MAX_CM;
uint16_t distLeft = DIST_MAX_CM;
uint16_t distRight = DIST_MAX_CM;

float yawDeg = 0.0f;
float gyroZBias = 0.0f;
uint32_t lastMpuMicros = 0;

volatile long encLeftTicks = 0;
volatile long encRightTicks = 0;
volatile int8_t encLeftSign = 1;
volatile int8_t encRightSign = 1;

float leftSpeedCms = 0.0f;
float rightSpeedCms = 0.0f;
float odomDistanceCm = 0.0f;

int currentLeftCmd = 0;
int currentRightCmd = 0;

uint32_t beepUntil = 0;

struct ButtonMemory {
  bool options = false;
  bool share = false;
  bool cross = false;
  bool circle = false;
  bool square = false;
  bool triangle = false;
};

ButtonMemory lastButton;

// ======================================================
// ISR
// ======================================================

void IRAM_ATTR onLeftEncoder() {
  encLeftTicks += encLeftSign;
}

void IRAM_ATTR onRightEncoder() {
  encRightTicks += encRightSign;
}

// ======================================================
// BASIC HELPERS
// ======================================================

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool risingEdge(bool now, bool &last) {
  bool edge = now && !last;
  last = now;
  return edge;
}

void beep(uint16_t ms) {
  digitalWrite(Pin::BUZZER, HIGH);
  beepUntil = millis() + ms;
}

void updateBuzzer() {
  if (beepUntil != 0 && millis() >= beepUntil) {
    digitalWrite(Pin::BUZZER, LOW);
    beepUntil = 0;
  }
}

//void setVacuum(bool on) {
//  vacuumOn = on;
//  if (ENABLE_VACUUM_OUTPUT) {
//    digitalWrite(Pin::VACUUM, on ? HIGH : LOW);
//  }
//}

// ======================================================
// PWM COMPATIBILITY LAYER
// Arduino-ESP32 v3: ledcAttach(pin, freq, resolution), ledcWrite(pin, duty)
// Arduino-ESP32 v2: ledcSetup(channel, freq, resolution), ledcAttachPin(pin, channel)
// ======================================================

void setupPwmPin(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// ======================================================
// MOTOR CONTROL
// ======================================================

int compensateMotorPwm(int absPwm) {
  absPwm = clampInt(absPwm, 0, MOTOR_MAX_PWM);
  if (absPwm == 0) return 0;

  // Map 1..255 menjadi MOTOR_MIN_PWM..255.
  return map(absPwm, 1, MOTOR_MAX_PWM, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}

void setOneMotor(
    uint8_t enPin,
    uint8_t pwmChannel,
    uint8_t in1,
    uint8_t in2,
    int speedPwm
) {
  speedPwm = clampInt(speedPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);

  if (speedPwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speedPwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Coast stop.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  uint8_t duty = static_cast<uint8_t>(compensateMotorPwm(abs(speedPwm)));
  writePwm(enPin, pwmChannel, duty);
}

void setDrive(int leftPwm, int rightPwm) {
  leftPwm = clampInt(leftPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  rightPwm = clampInt(rightPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);

  currentLeftCmd = leftPwm;
  currentRightCmd = rightPwm;

  encLeftSign = (leftPwm >= 0) ? 1 : -1;
  encRightSign = (rightPwm >= 0) ? 1 : -1;

  setOneMotor(Pin::LEFT_EN, PWM_CH_LEFT, Pin::LEFT_IN1, Pin::LEFT_IN2, leftPwm);
  setOneMotor(Pin::RIGHT_EN, PWM_CH_RIGHT, Pin::RIGHT_IN1, Pin::RIGHT_IN2, rightPwm);
}

void stopDrive() {
  setDrive(0, 0);
}

// ======================================================
// ULTRASONIC
// ======================================================

uint16_t readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration == 0) {
    return DIST_MAX_CM;
  }

  // HC-SR04 standard approximation: cm = microseconds / 58.
  uint16_t cm = static_cast<uint16_t>(duration / 58UL);

  if (cm > DIST_MAX_CM) cm = DIST_MAX_CM;
  return cm;
}

void updateDistances() {
  static uint32_t lastPingMs = 0;
  static uint8_t pingIndex = 0;

  const uint32_t now = millis();

  // Baca sensor bergantian agar ultrasonic tidak saling mengganggu.
  if (now - lastPingMs < 55) return;
  lastPingMs = now;

  if (pingIndex == 0) {
    distFront = readUltrasonicCm(Pin::TRIG_FRONT, Pin::ECHO_FRONT);
  } else if (pingIndex == 1) {
    distLeft = readUltrasonicCm(Pin::TRIG_LEFT, Pin::ECHO_LEFT);
  } else {
    distRight = readUltrasonicCm(Pin::TRIG_RIGHT, Pin::ECHO_RIGHT);
  }

  pingIndex = (pingIndex + 1) % 3;
}

// ======================================================
// MPU6050
// ======================================================

void calibrateMpuGyro() {
  if (!hasMpu) return;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  constexpr int samples = 200;
  float sumZ = 0.0f;

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    sumZ += gyro.gyro.z;
    delay(3);
  }

  gyroZBias = sumZ / samples;
  lastMpuMicros = micros();
}

void updateMpuYaw() {
  if (!hasMpu) return;

  uint32_t now = micros();
  float dt = (now - lastMpuMicros) / 1000000.0f;
  lastMpuMicros = now;

  if (dt <= 0.0f || dt > 0.2f) return;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  // gyro.gyro.z dari Adafruit_MPU6050 dalam rad/s.
  const float radToDeg = 57.2957795f;
  yawDeg += (gyro.gyro.z - gyroZBias) * radToDeg * dt;

  if (yawDeg > 180.0f) yawDeg -= 360.0f;
  if (yawDeg < -180.0f) yawDeg += 360.0f;
}

// ======================================================
// ENCODER / ODOMETRY
// ======================================================

void resetOdom() {
  noInterrupts();
  encLeftTicks = 0;
  encRightTicks = 0;
  interrupts();

  leftSpeedCms = 0.0f;
  rightSpeedCms = 0.0f;
  odomDistanceCm = 0.0f;
  yawDeg = 0.0f;
}

void updateOdom() {
  static uint32_t lastMs = 0;
  static long lastLeft = 0;
  static long lastRight = 0;

  uint32_t now = millis();
  if (now - lastMs < 500) return;

  long leftNow;
  long rightNow;

  noInterrupts();
  leftNow = encLeftTicks;
  rightNow = encRightTicks;
  interrupts();

  long dLeft = leftNow - lastLeft;
  long dRight = rightNow - lastRight;

  uint32_t dtMs = now - lastMs;
  if (lastMs == 0) dtMs = 500;

  leftSpeedCms = (dLeft * CM_PER_TICK * 1000.0f) / dtMs;
  rightSpeedCms = (dRight * CM_PER_TICK * 1000.0f) / dtMs;

  odomDistanceCm = ((leftNow + rightNow) * 0.5f) * CM_PER_TICK;

  lastLeft = leftNow;
  lastRight = rightNow;
  lastMs = now;
}

// ======================================================
// PS4 / DS4 CONTROL
// ======================================================

void updatePs4ConnectionState() {
  bool connected = PS4.isConnected();

  if (connected && !ps4WasConnected) {
    driveMode = DriveMode::MANUAL;
    emergencyStop = false;

    PS4.setLed(0, 0, 80);
    PS4.setRumble(0, 60);
    PS4.sendToController();

    beep(150);
    Serial.println("[DS4] Connected. Manual mode active.");
  }

  if (!connected && ps4WasConnected) {
    lastButton = ButtonMemory();

    if (driveMode == DriveMode::MANUAL) {
      stopDrive();
      driveMode = DriveMode::AUTO;
    }

    beep(250);
    Serial.println("[DS4] Disconnected. Back to AUTO mode.");
  }

  ps4WasConnected = connected;
}

int applyStickDeadband(int value, int deadband = 12) {
  if (abs(value) < deadband) return 0;
  return value;
}

void handleDs4Buttons() {
  if (!PS4.isConnected()) return;

  if (risingEdge(PS4.Options(), lastButton.options)) {
    emergencyStop = false;
    driveMode = (driveMode == DriveMode::AUTO) ? DriveMode::MANUAL : DriveMode::AUTO;
    beep(100);
  }

  if (risingEdge(PS4.Share(), lastButton.share)) {
    emergencyStop = false;
    driveMode = DriveMode::AUTO;
    beep(80);
  }

  if (risingEdge(PS4.Circle(), lastButton.circle)) {
    emergencyStop = !emergencyStop;
    stopDrive();
    beep(emergencyStop ? 350 : 120);
  }
/*
  if (risingEdge(PS4.Cross(), lastButton.cross)) {
    setVacuum(!vacuumOn);
    beep(90);
  }
*/
  if (risingEdge(PS4.Square(), lastButton.square)) {
    resetOdom();
    beep(90);
  }

  if (risingEdge(PS4.Triangle(), lastButton.triangle)) {
    beep(220);
  }
}

void updateManualMode() {
  if (!PS4.isConnected()) {
    stopDrive();
    return;
  }

  if (emergencyStop) {
    stopDrive();
    return;
  }

  int throttle = applyStickDeadband(PS4.LStickY());
  int turn = applyStickDeadband(PS4.RStickX());

  // Safety guard: kalau depan sangat dekat, manual tidak boleh maju.
  if (distFront <= MANUAL_FRONT_GUARD_CM && throttle > 0) {
    throttle = 0;
    beep(60);
  }

  // Trigger untuk scaling speed.
  // R2 = lebih cepat, L2 = mode halus.
  float speedScale = 0.65f + (PS4.R2Value() / 255.0f) * 0.35f;

  if (PS4.L2()) {
    speedScale *= 0.45f;
  }

  // Tombol L1/R1 untuk rotate in place.
  if (PS4.L1()) {
    setDrive(-130, 130);
    return;
  }

  if (PS4.R1()) {
    setDrive(130, -130);
    return;
  }

  // D-pad fallback.
  if (PS4.Up()) {
    setDrive(140, 140);
    return;
  }

  if (PS4.Down()) {
    setDrive(-120, -120);
    return;
  }

  if (PS4.Left()) {
    setDrive(-110, 110);
    return;
  }

  if (PS4.Right()) {
    setDrive(110, -110);
    return;
  }

  // Arcade drive:
  // Left stick Y = maju/mundur.
  // Right stick X = belok.
  int left = throttle + turn;
  int right = throttle - turn;

  left = clampInt(left, -128, 127);
  right = clampInt(right, -128, 127);

  int leftPwm = static_cast<int>((left / 127.0f) * MOTOR_MAX_PWM * speedScale);
  int rightPwm = static_cast<int>((right / 127.0f) * MOTOR_MAX_PWM * speedScale);

  setDrive(leftPwm, rightPwm);
}

// ======================================================
// AUTO OBSTACLE AVOIDANCE
// ======================================================

void enterAutoState(AutoState state, uint32_t durationMs) {
  autoState = state;
  autoStateUntil = millis() + durationMs;
}

void updateAutoMode() {
  if (emergencyStop) {
    stopDrive();
    return;
  }

  const uint32_t now = millis();

  if (autoState == AutoState::BACKUP) {
    setDrive(-AUTO_REVERSE_PWM, -AUTO_REVERSE_PWM);

    if (now >= autoStateUntil) {
      if (distLeft >= distRight) {
        enterAutoState(AutoState::TURN_LEFT, AUTO_TURN_MS);
      } else {
        enterAutoState(AutoState::TURN_RIGHT, AUTO_TURN_MS);
      }
    }

    return;
  }

  if (autoState == AutoState::TURN_LEFT) {
    setDrive(-AUTO_TURN_PWM, AUTO_TURN_PWM);

    if (now >= autoStateUntil) {
      autoState = AutoState::CRUISE;
    }

    return;
  }

  if (autoState == AutoState::TURN_RIGHT) {
    setDrive(AUTO_TURN_PWM, -AUTO_TURN_PWM);

    if (now >= autoStateUntil) {
      autoState = AutoState::CRUISE;
    }

    return;
  }

  // CRUISE decision
  bool frontBlocked = distFront <= FRONT_STOP_CM;
  bool leftBlocked = distLeft <= SIDE_STOP_CM;
  bool rightBlocked = distRight <= SIDE_STOP_CM;

  if (frontBlocked) {
    beep(90);
    enterAutoState(AutoState::BACKUP, AUTO_BACKUP_MS);
    setDrive(-AUTO_REVERSE_PWM, -AUTO_REVERSE_PWM);
    return;
  }

  if (leftBlocked && !rightBlocked) {
    // Terlalu dekat kiri, steer ke kanan.
    setDrive(AUTO_STEER_FAST_PWM, AUTO_STEER_SLOW_PWM);
    return;
  }

  if (rightBlocked && !leftBlocked) {
    // Terlalu dekat kanan, steer ke kiri.
    setDrive(AUTO_STEER_SLOW_PWM, AUTO_STEER_FAST_PWM);
    return;
  }

  setDrive(AUTO_FORWARD_PWM, AUTO_FORWARD_PWM);
}

// ======================================================
// OLED
// ======================================================

const char *modeText() {
  if (emergencyStop) return "STOP";
  return (driveMode == DriveMode::AUTO) ? "AUTO" : "DS4";
}

const char *autoStateText() {
  switch (autoState) {
    case AutoState::CRUISE: return "CRS";
    case AutoState::BACKUP: return "BAK";
    case AutoState::TURN_LEFT: return "TL";
    case AutoState::TURN_RIGHT: return "TR";
    default: return "---";
  }
}

void updateDisplay() {
  if (!hasDisplay) return;

  static uint32_t lastDrawMs = 0;
  uint32_t now = millis();

  if (now - lastDrawMs < 250) return;
  lastDrawMs = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(modeText());
  display.print(" ");
  display.print(PS4.isConnected() ? "BT" : "--");

  display.setCursor(0, 8);
  display.print("F:");
  display.print(distFront);
  display.print(" L:");
  display.print(distLeft);
  display.print(" R:");
  display.print(distRight);

  display.setCursor(0, 16);
  display.print("Sp:");
  display.print((leftSpeedCms + rightSpeedCms) * 0.5f, 0);
  display.print("cm/s D:");
  display.print(odomDistanceCm / 100.0f, 1);
  display.print("m");

  display.setCursor(0, 24);
  display.print("Yaw:");
  display.print(yawDeg, 0);
  display.print(" ");
  display.print(autoStateText());

  display.display();
}

void displaySplash(const char *line1, const char *line2) {
  if (!hasDisplay) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(line1);
  display.setCursor(0, 12);
  display.print(line2);
  display.display();
}

// ======================================================
// SETUP
// ======================================================

void setupPins() {
  pinMode(Pin::LEFT_IN1, OUTPUT);
  pinMode(Pin::LEFT_IN2, OUTPUT);
  pinMode(Pin::RIGHT_IN1, OUTPUT);
  pinMode(Pin::RIGHT_IN2, OUTPUT);

  setupPwmPin(Pin::LEFT_EN, PWM_CH_LEFT);
  setupPwmPin(Pin::RIGHT_EN, PWM_CH_RIGHT);

  pinMode(Pin::TRIG_FRONT, OUTPUT);
  pinMode(Pin::TRIG_LEFT, OUTPUT);
  pinMode(Pin::TRIG_RIGHT, OUTPUT);

  digitalWrite(Pin::TRIG_FRONT, LOW);
  digitalWrite(Pin::TRIG_LEFT, LOW);
  digitalWrite(Pin::TRIG_RIGHT, LOW);

  pinMode(Pin::ECHO_FRONT, INPUT);
  pinMode(Pin::ECHO_LEFT, INPUT);
  pinMode(Pin::ECHO_RIGHT, INPUT);

  // GPIO39 tidak punya internal pull-up. Pastikan encoder module punya pull-up sendiri.
  pinMode(Pin::ENC_LEFT_A, INPUT);
  pinMode(Pin::ENC_RIGHT_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Pin::ENC_LEFT_A), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin::ENC_RIGHT_A), onRightEncoder, RISING);

  pinMode(Pin::BUZZER, OUTPUT);
  digitalWrite(Pin::BUZZER, LOW);

  //if (ENABLE_VACUUM_OUTPUT) {
  //  pinMode(Pin::VACUUM, OUTPUT);
  //  digitalWrite(Pin::VACUUM, LOW);
  //}

  stopDrive();
}

void setupDisplayAndMpu() {
  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL);

  hasDisplay = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  if (hasDisplay) {
    displaySplash("VacBot ESP32", "Booting...");
  } else {
    Serial.println("[OLED] Not found.");
  }

  hasMpu = mpu.begin();

  if (hasMpu) {
    Serial.println("[MPU6050] Found.");

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    displaySplash("MPU6050 OK", "Calibrating...");
    calibrateMpuGyro();
  } else {
    Serial.println("[MPU6050] Not found. Continue without MPU.");
  }
}

void setupPs4() {
  if (std::strlen(DS4_HOST_MAC) > 0) {
    PS4.begin(DS4_HOST_MAC);
    Serial.print("[DS4] PS4.begin with MAC: ");
    Serial.println(DS4_HOST_MAC);
  } else {
    PS4.begin();
    Serial.println("[DS4] PS4.begin with default ESP32 Bluetooth MAC.");
  }

  Serial.print("[DS4] ESP32 Bluetooth address: ");
  Serial.println(PS4.getAddress());
  Serial.println("[DS4] Pair DS4 to this address if controller cannot connect.");
  Serial.println("[DS4] Controls:");
  Serial.println("  OPTIONS : toggle AUTO/DS4");
  Serial.println("  SHARE   : force AUTO");
  Serial.println("  CIRCLE  : emergency stop toggle");
  Serial.println("  CROSS   : vacuum toggle");
  Serial.println("  SQUARE  : reset odometry/yaw");
  Serial.println("  LStickY : throttle");
  Serial.println("  RStickX : steering");
  Serial.println("  L2      : slow mode");
  Serial.println("  R2      : boost");
  Serial.println("  L1/R1   : rotate in place");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== VacBot ESP32 PlatformIO ===");

  setupPins();
  setupDisplayAndMpu();
  setupPs4();

  //setVacuum(vacuumOn);

  beep(150);
  displaySplash("VacBot Ready", "AUTO mode");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  updateBuzzer();

  updateDistances();
  updateMpuYaw();
  updateOdom();

  updatePs4ConnectionState();
  handleDs4Buttons();

  if (emergencyStop) {
    stopDrive();
  } else if (driveMode == DriveMode::MANUAL && PS4.isConnected()) {
    updateManualMode();
  } else {
    driveMode = DriveMode::AUTO;
    updateAutoMode();
  }

  updateDisplay();
}