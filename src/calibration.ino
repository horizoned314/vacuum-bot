#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= MPU =================
MPU6050 mpu;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

// ================= HC-SR04 =================
#define TRIG_FRONT 19
#define ECHO_FRONT 18
#define TRIG_LEFT 25
#define ECHO_LEFT 26
#define TRIG_RIGHT 27
#define ECHO_RIGHT 33

// ================= ENCODER =================
#define ENC_LEFT 35
#define ENC_RIGHT 34

volatile long countL = 0;
volatile long countR = 0;

// ================= MOTOR =================
#define ENA 32
#define IN1 14
#define IN2 13
#define IN3 2
#define IN4 15
#define ENB 4

// ================= BUZZER =================
#define BUZZER 23

float motorBalanceFactor = 1.0;

// ================= COMMAND =================
bool requestRecalibration = false;

// ================= INTERRUPTS =================
void IRAM_ATTR isrLeft() { countL++; }
void IRAM_ATTR isrRight() { countR++; }

// ================= BUZZ =================
void beep(int t = 100) {
  digitalWrite(BUZZER, HIGH);
  delay(t);
  digitalWrite(BUZZER, LOW);
}

// ================= SERIAL COMMAND =================
void checkSerialCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      requestRecalibration = true;
    }
  }
}

void resetCalibration() {
  Serial.println("=== RESET CALIBRATION ===");

  beep(200);
  delay(300);
  beep(200);

  calibrateMPU();
  delay(500);
  calibrateMotors();

  Serial.println("=== DONE ===");
}

// ================= ULTRASONIC =================
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ================= MOTOR =================
void setMotorRaw(int leftPWM, int rightPWM) {
  digitalWrite(IN1, leftPWM > 0);
  digitalWrite(IN2, leftPWM <= 0);
  ledcWrite(ENA, abs(leftPWM));

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, abs(rightPWM));
}

// ================= MPU CALIB =================
void calibrateMPU() {
  gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
  accelOffsetX = accelOffsetY = accelOffsetZ = 0;

  int samples = 3000;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;

    gyroOffsetX += gx;
    gyroOffsetY += gy;
    gyroOffsetZ += gz;

    delay(1);
  }

  accelOffsetX /= samples;
  accelOffsetY /= samples;
  accelOffsetZ /= samples;

  gyroOffsetX /= samples;
  gyroOffsetY /= samples;
  gyroOffsetZ /= samples;

  Serial.println("=== MPU CALIBRATION RESULT ===");

  Serial.print("Accel Offset: ");
  Serial.print(accelOffsetX); Serial.print(", ");
  Serial.print(accelOffsetY); Serial.print(", ");
  Serial.println(accelOffsetZ);

  Serial.print("Gyro Offset: ");
  Serial.print(gyroOffsetX); Serial.print(", ");
  Serial.print(gyroOffsetY); Serial.print(", ");
  Serial.println(gyroOffsetZ);

  beep(200);
}

// ================= MOTOR CALIB =================
void calibrateMotors() {
  countL = 0;
  countR = 0;

  setMotorRaw(120, 120);
  delay(1000);

  countL = 0;
  countR = 0;

  unsigned long start = millis();

  setMotorRaw(150, 150);

  while (millis() - start < 3000);

  setMotorRaw(0, 0);

  Serial.print("L: "); Serial.print(countL);
  Serial.print(" | R: "); Serial.println(countR);

  if (countR > 0) {
    motorBalanceFactor = (float)countL / (float)countR;
  }

  Serial.print("Balance Factor: ");
  Serial.println(motorBalanceFactor, 5);

  beep(100);
  delay(100);
  beep(100);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(BUZZER, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  mpu.initialize();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(ENC_LEFT, INPUT);
  pinMode(ENC_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), isrRight, RISING);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  delay(2000);

  beep();
  calibrateMPU();
  delay(1000);
  calibrateMotors();
}

// ================= LOOP =================
void loop() {

  checkSerialCommand();

  if (requestRecalibration) {
    requestRecalibration = false;
    resetCalibration();
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axc = ax - accelOffsetX;
  float ayc = ay - accelOffsetY;
  float azc = az - accelOffsetZ;

  float gzc = (gz - gyroOffsetZ) / 131.0;

  static float gzFiltered = 0;
  gzFiltered = 0.9 * gzFiltered + 0.1 * gzc;

  long f = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long l = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long r = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  display.clearDisplay();
  display.setCursor(0, 0);

  display.printf("F:%ld L:%ld R:%ld\n", f, l, r);
  display.printf("GZ:%.1f Bal:%.2f\n", gzFiltered, motorBalanceFactor);

  display.display();

  Serial.print("F:"); Serial.print(f);
  Serial.print(" L:"); Serial.print(l);
  Serial.print(" R:"); Serial.print(r);
  Serial.print(" | GZ: "); Serial.print(gzFiltered);
  Serial.print(" | Bal: "); Serial.println(motorBalanceFactor, 3);

  delay(200);
}