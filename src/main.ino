/*
 * VacBot V2 - ESP32 Firmware (Versi Diperbaiki)
 * ===============================================
 * Cocok dengan hardware list:
 *   - ESP32 DevKit V1 (30 Pin)
 *   - OLED SSD1306 128x32 I2C          ← DITAMBAHKAN
 *   - MPU-6050 (Gyro + Accel)
 *   - 3x HC-SR04 Ultrasonic            ← FIX: non-blocking
 *   - 2x Rotary Encoder Module          ← FIX: 2 pin per encoder (CLK+DT)
 *   - L298N Motor Driver
 *   - Active Buzzer 5V                  ← DITAMBAHKAN
 *
 * Library yang dibutuhkan (install via Library Manager):
 *   - ArduinoJson         (Benoit Blanchon)
 *   - MPU6050             (Electronic Cats atau jrowberg)
 *   - Adafruit SSD1306    (Adafruit)
 *   - Adafruit GFX        (Adafruit) -- dependensi SSD1306
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <MPU6050.h>
#include <Adafruit_SSD1306.h>

// ═══════════════════════════════════════════════
// WiFi
// ═══════════════════════════════════════════════
const char* WIFI_SSID     = "NAMA_WIFI_ANDA";
const char* WIFI_PASSWORD = "PASSWORD_WIFI_ANDA";

IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress PC_IP(192, 168, 1, 10);   // ← ganti dengan IP PC Anda

const uint16_t UDP_PORT_SEND    = 4210; // ESP32 → PC
const uint16_t UDP_PORT_RECEIVE = 4211; // PC   → ESP32

// ═══════════════════════════════════════════════
// OLED SSD1306 (128x32, I2C)
// Catatan: I2C share Wire dengan MPU6050
//   SDA → GPIO 21
//   SCL → GPIO 22
// ═══════════════════════════════════════════════
#define OLED_WIDTH    128
#define OLED_HEIGHT    32
#define OLED_I2C_ADDR 0x3C   // Biasanya 0x3C untuk 128x32

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ═══════════════════════════════════════════════
// Active Buzzer
// Active buzzer: HIGH = bunyi, LOW = diam
// ═══════════════════════════════════════════════
#define BUZZER_PIN  23

// ═══════════════════════════════════════════════
// Motor Driver L298N
//
// ⚠️ PERHATIAN GPIO 12:
//   GPIO 12 (MTDI) pada ESP32 mempengaruhi flash voltage saat boot.
//   Jika robot tidak mau boot, pindah ke GPIO 15.
//   Solusi terbaik: gunakan pin yang aman.
//
// Left Motor  → IN1=25, IN2=26, ENA=27
// Right Motor → IN3=32, IN4=33, ENB=14
// (GPIO 12 dihindari, diganti GPIO 32/33/14)
// ═══════════════════════════════════════════════
#define LEFT_IN1   25
#define LEFT_IN2   26
#define LEFT_ENA   27    // PWM

#define RIGHT_IN3  32
#define RIGHT_IN4  33
#define RIGHT_ENB  14    // PWM  (bukan 13 karena 13 dekat bootstrap)

#define LEFT_PWM_CH  0
#define RIGHT_PWM_CH 1
#define PWM_FREQ     1000
#define PWM_RES      8

// ═══════════════════════════════════════════════
// HC-SR04 Ultrasonic
// ECHO HC-SR04 output 5V → pakai voltage divider ke ESP32 3.3V
//   1kΩ + 2kΩ: ECHO → 1kΩ → GPIO → 2kΩ → GND
// ═══════════════════════════════════════════════
#define TRIG_FRONT 16
#define ECHO_FRONT 17

#define TRIG_LEFT  18
#define ECHO_LEFT  19

#define TRIG_RIGHT  4
#define ECHO_RIGHT  5

// ═══════════════════════════════════════════════
// Rotary Encoder Module — 2 pin per encoder
//
// Rotary encoder module punya: CLK (A), DT (B), SW (opsional)
// Kita butuh CLK + DT untuk deteksi arah putaran.
// Total 4 pin interrupt untuk 2 encoder.
//
// GPIO interrupt-capable yang aman di DevKit V1 30 Pin:
//   Left:  CLK=34, DT=35   (input-only, aman untuk ISR)
//   Right: CLK=36, DT=39   (input-only = VP & VN, tanpa pull-up internal)
//
// ⚠️ GPIO 36 & 39 TIDAK punya internal pull-up.
//    Pasang resistor pull-up eksternal 10kΩ ke 3.3V pada CLK & DT kanan.
// ═══════════════════════════════════════════════
#define ENC_LEFT_CLK  34    // Input only
#define ENC_LEFT_DT   35    // Input only
#define ENC_RIGHT_CLK 36    // Input only, VP — BUTUH pull-up eksternal
#define ENC_RIGHT_DT  39    // Input only, VN — BUTUH pull-up eksternal

// ═══════════════════════════════════════════════
// Konstanta
// ═══════════════════════════════════════════════
#define OBSTACLE_DISTANCE_CM  20
#define COMMAND_TIMEOUT_MS    1000
#define SEND_INTERVAL_MS      100    // UDP 10 Hz
#define OLED_INTERVAL_MS      250    // Refresh OLED 4 Hz
#define ULTRA_INTERVAL_MS     50     // Ukur ultrasonic tiap 50ms per sensor

// ═══════════════════════════════════════════════
// Global State
// ═══════════════════════════════════════════════
WiFiUDP    udp;
MPU6050    mpu;

// Encoder: volatile karena diupdate dari ISR
volatile long enc_left  = 0;
volatile long enc_right = 0;

// Simpan state CLK sebelumnya untuk deteksi arah
volatile bool lastCLK_left  = false;
volatile bool lastCLK_right = false;

// Sensor readings
float yaw_rad     = 0.0f;
float ultra_front = 999.0f;
float ultra_left  = 999.0f;
float ultra_right = 999.0f;

// Timing
unsigned long lastSendTime    = 0;
unsigned long lastOLEDTime    = 0;
unsigned long lastCommandTime = 0;
unsigned long lastMPUTime     = 0;

// Ultrasonic non-blocking state machine
// Setiap sensor punya giliran (round-robin) agar tidak blocking
uint8_t       ultraState     = 0;   // 0=front, 1=left, 2=right
unsigned long ultraTrigTime  = 0;
unsigned long ultraLastMeasure = 0;
bool          ultraWaiting   = false;

// Gyro
float gyroZ_offset = 0.0f;

// Motor
String currentCommand = "stop";
int    currentSpeed   = 0;

// Buzzer non-blocking
unsigned long buzzerEndTime = 0;
bool          buzzerActive  = false;

// ═══════════════════════════════════════════════
// ISR Encoder
// ═══════════════════════════════════════════════
void IRAM_ATTR onEncoderLeftCLK() {
  // Baca DT saat CLK rising edge untuk deteksi arah
  bool dt = digitalRead(ENC_LEFT_DT);
  if (dt == HIGH) enc_left++;   // Maju
  else            enc_left--;   // Mundur
}

void IRAM_ATTR onEncoderRightCLK() {
  bool dt = digitalRead(ENC_RIGHT_DT);
  if (dt == HIGH) enc_right++;
  else            enc_right--;
}

// ═══════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[VacBot V2] Booting...");

  setupBuzzer();
  setupOLED();
  setupMotors();
  setupUltrasonics();
  setupEncoders();
  setupMPU();
  setupWiFi();
  setupUDP();

  lastMPUTime = micros();

  // Bunyi 2x tanda siap
  beep(100); delay(150); beep(100);

  Serial.println("[VacBot V2] Ready!");
  updateOLED("VacBot V2", "Ready!", "", "");
}

// ═══════════════════════════════════════════════
// MAIN LOOP — Non-blocking
// ═══════════════════════════════════════════════
void loop() {
  readMPU();
  readUltrasonicsNonBlocking();  // State machine, tidak blocking
  receiveUDP();
  updateSafety();
  controlMotors();
  updateBuzzer();

  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    sendUDP();
    lastSendTime = millis();
  }

  if (millis() - lastOLEDTime >= OLED_INTERVAL_MS) {
    refreshOLED();
    lastOLEDTime = millis();
  }
}

// ═══════════════════════════════════════════════
// SETUP HELPERS
// ═══════════════════════════════════════════════

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("[Buzzer] OK");
}

void setupOLED() {
  // Wire.begin() dipanggil sekali, share dengan MPU6050
  Wire.begin(21, 22);   // SDA=21, SCL=22

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("[OLED] GAGAL — cek wiring!");
    // Lanjutkan meski OLED gagal (non-critical)
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("VacBot V2");
  oled.println("Booting...");
  oled.display();
  Serial.println("[OLED] OK");
}

void setupMotors() {
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  ledcSetup(LEFT_PWM_CH,  PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA,  LEFT_PWM_CH);
  ledcAttachPin(RIGHT_ENB, RIGHT_PWM_CH);

  stopMotors();
  Serial.println("[Motor] OK");
}

void setupUltrasonics() {
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  // Pastikan TRIG LOW
  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_LEFT,  LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  Serial.println("[Ultrasonic] OK");
}

void setupEncoders() {
  // GPIO 34 & 35: input-only, punya internal pull-up
  pinMode(ENC_LEFT_CLK, INPUT_PULLUP);
  pinMode(ENC_LEFT_DT,  INPUT_PULLUP);

  // GPIO 36 & 39: input-only, TIDAK ada internal pull-up
  // → pasang resistor 10kΩ ke 3.3V secara eksternal
  pinMode(ENC_RIGHT_CLK, INPUT);
  pinMode(ENC_RIGHT_DT,  INPUT);

  // ISR pada rising edge CLK saja (lebih stabil dari CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_CLK),  onEncoderLeftCLK,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_CLK), onEncoderRightCLK, RISING);

  Serial.println("[Encoder] OK");
}

void setupMPU() {
  // Wire sudah begin() di setupOLED, jangan panggil lagi
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[MPU6050] GAGAL — cek wiring SDA/SCL!");
    beepError();
  } else {
    Serial.println("[MPU6050] OK");
    calibrateGyro();
  }
}

void calibrateGyro() {
  Serial.print("[MPU6050] Kalibrasi gyro (jangan gerakkan robot)... ");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += mpu.getRotationZ();
    delay(2);
  }
  gyroZ_offset = (float)sum / 500.0f;
  Serial.printf("offset = %.2f raw\n", gyroZ_offset);
}

void setupWiFi() {
  updateOLED("WiFi", "Connecting...", WIFI_SSID, "");

  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[WiFi] Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] OK! IP: %s\n", WiFi.localIP().toString().c_str());
    updateOLED("WiFi OK", WiFi.localIP().toString().c_str(), "", "");
    beep(200);
  } else {
    Serial.println("\n[WiFi] GAGAL! Periksa SSID/Password.");
    updateOLED("WiFi GAGAL", "Cek SSID", "& Password", "");
    beepError();
  }
}

void setupUDP() {
  udp.begin(UDP_PORT_RECEIVE);
  Serial.printf("[UDP] Listen port %d\n", UDP_PORT_RECEIVE);
}

// ═══════════════════════════════════════════════
// SENSOR — MPU6050
// ═══════════════════════════════════════════════
void readMPU() {
  unsigned long now = micros();
  float dt = (now - lastMPUTime) / 1e6f;
  lastMPUTime = now;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Konversi raw → deg/s (range ±250°/s → bagi 131)
  float gyroZ_dps = (gz - gyroZ_offset) / 131.0f;

  // Integrasi: yaw += ω * dt (dalam radian)
  yaw_rad += (gyroZ_dps * (PI / 180.0f)) * dt;

  // Normalisasi ke [-π, π]
  while (yaw_rad >  PI) yaw_rad -= 2.0f * PI;
  while (yaw_rad < -PI) yaw_rad += 2.0f * PI;
}

// ═══════════════════════════════════════════════
// SENSOR — HC-SR04 Non-Blocking (State Machine)
//
// Masalah versi lama:
//   pulseIn() memblokir CPU selama ~17ms per sensor
//   3 sensor × 17ms = ~51ms blocking per siklus → loop melambat drastis
//
// Solusi: state machine round-robin
//   State 0: trigger FRONT, tunggu echo
//   State 1: trigger LEFT, tunggu echo
//   State 2: trigger RIGHT, tunggu echo
//   Setiap state ganti tiap ULTRA_INTERVAL_MS
// ═══════════════════════════════════════════════
void readUltrasonicsNonBlocking() {
  unsigned long now = millis();

  // Hanya mulai pengukuran baru setiap ULTRA_INTERVAL_MS
  if (!ultraWaiting && (now - ultraLastMeasure >= ULTRA_INTERVAL_MS)) {
    ultraLastMeasure = now;

    // Pilih sensor berdasarkan state
    int trigPin = (ultraState == 0) ? TRIG_FRONT :
                  (ultraState == 1) ? TRIG_LEFT  : TRIG_RIGHT;

    // Kirim 10µs pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    ultraTrigTime = micros();
    ultraWaiting  = true;
  }

  // Jika sedang menunggu echo
  if (ultraWaiting) {
    int echoPin = (ultraState == 0) ? ECHO_FRONT :
                  (ultraState == 1) ? ECHO_LEFT  : ECHO_RIGHT;

    // Cek apakah echo sudah HIGH (mulai echo)
    if (digitalRead(echoPin) == HIGH) {
      // Tunggu echo selesai (LOW) dengan timeout non-blocking
      unsigned long echoStart = micros();
      while (digitalRead(echoPin) == HIGH) {
        if (micros() - echoStart > 25000) break;   // timeout 25ms
        yield();  // Beri CPU untuk task lain
      }
      float dist = (micros() - echoStart) * 0.01715f;  // cm

      // Simpan ke variabel yang sesuai
      if      (ultraState == 0) ultra_front = dist;
      else if (ultraState == 1) ultra_left  = dist;
      else                      ultra_right = dist;

      // Pindah ke sensor berikutnya
      ultraState   = (ultraState + 1) % 3;
      ultraWaiting = false;
    }
    // Timeout total: jika terlalu lama tidak ada echo
    else if (micros() - ultraTrigTime > 30000) {
      if      (ultraState == 0) ultra_front = 999.0f;
      else if (ultraState == 1) ultra_left  = 999.0f;
      else                      ultra_right = 999.0f;

      ultraState   = (ultraState + 1) % 3;
      ultraWaiting = false;
    }
  }
}

// ═══════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════
void moveForward(int speed) {
  digitalWrite(LEFT_IN1,  HIGH); digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  ledcWrite(LEFT_PWM_CH, speed); ledcWrite(RIGHT_PWM_CH, speed);
}

void moveBackward(int speed) {
  digitalWrite(LEFT_IN1,  LOW);  digitalWrite(LEFT_IN2,  HIGH);
  digitalWrite(RIGHT_IN3, LOW);  digitalWrite(RIGHT_IN4, HIGH);
  ledcWrite(LEFT_PWM_CH, speed); ledcWrite(RIGHT_PWM_CH, speed);
}

void turnLeft(int speed) {
  // Kiri mundur, kanan maju → robot berputar ke kiri
  digitalWrite(LEFT_IN1,  LOW);  digitalWrite(LEFT_IN2,  HIGH);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  ledcWrite(LEFT_PWM_CH, speed); ledcWrite(RIGHT_PWM_CH, speed);
}

void turnRight(int speed) {
  // Kiri maju, kanan mundur → robot berputar ke kanan
  digitalWrite(LEFT_IN1,  HIGH); digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN3, LOW);  digitalWrite(RIGHT_IN4, HIGH);
  ledcWrite(LEFT_PWM_CH, speed); ledcWrite(RIGHT_PWM_CH, speed);
}

void stopMotors() {
  digitalWrite(LEFT_IN1,  LOW); digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, LOW);
  ledcWrite(LEFT_PWM_CH, 0);    ledcWrite(RIGHT_PWM_CH, 0);
}

void controlMotors() {
  if      (currentCommand == "forward")  moveForward(currentSpeed);
  else if (currentCommand == "backward") moveBackward(currentSpeed);
  else if (currentCommand == "left")     turnLeft(currentSpeed);
  else if (currentCommand == "right")    turnRight(currentSpeed);
  else                                   stopMotors();
}

// ═══════════════════════════════════════════════
// BUZZER — Non-blocking
// Gunakan beep() dari mana saja, tidak akan blocking loop
// ═══════════════════════════════════════════════
void beep(int durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerEndTime  = millis() + durationMs;
  buzzerActive   = true;
}

void beepError() {
  // 3 bunyi pendek
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);
  }
}

void updateBuzzer() {
  // Matikan buzzer setelah waktu habis
  if (buzzerActive && millis() >= buzzerEndTime) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }
}

// ═══════════════════════════════════════════════
// SAFETY
// ═══════════════════════════════════════════════
void updateSafety() {
  // Aturan 1: Obstacle depan saat maju → berhenti + bunyi
  if (ultra_front < OBSTACLE_DISTANCE_CM && currentCommand == "forward") {
    Serial.printf("[SAFETY] Obstacle %.1f cm! Stop.\n", ultra_front);
    currentCommand = "stop";
    beep(50);   // Bunyi singkat peringatan
  }

  // Aturan 2: Timeout perintah → berhenti
  if ((millis() - lastCommandTime) > COMMAND_TIMEOUT_MS && currentCommand != "stop") {
    Serial.println("[SAFETY] Timeout. Stop.");
    currentCommand = "stop";
  }
}

// ═══════════════════════════════════════════════
// UDP
// ═══════════════════════════════════════════════
void sendUDP() {
  StaticJsonDocument<256> doc;
  doc["yaw"]         = yaw_rad;
  doc["enc_left"]    = enc_left;
  doc["enc_right"]   = enc_right;
  doc["ultra_front"] = ultra_front;
  doc["ultra_left"]  = ultra_left;
  doc["ultra_right"] = ultra_right;

  char buf[256];
  size_t len = serializeJson(doc, buf);
  udp.beginPacket(PC_IP, UDP_PORT_SEND);
  udp.write((uint8_t*)buf, len);
  udp.endPacket();
}

void receiveUDP() {
  int pktSize = udp.parsePacket();
  if (pktSize == 0) return;

  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, buf)) return;   // Parse error → abaikan

  if (doc.containsKey("move")) {
    currentCommand  = doc["move"].as<String>();
    currentSpeed    = doc["speed"] | 150;
    lastCommandTime = millis();
  }
}

// ═══════════════════════════════════════════════
// OLED — Display informasi real-time
// Layout 128x32:
//   Baris 0 (y=0):  Status / Judul
//   Baris 1 (y=10): Info utama
//   Baris 2 (y=20): Info tambahan
// ═══════════════════════════════════════════════
void updateOLED(String line0, String line1, String line2, String line3) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0,  0); oled.print(line0);
  oled.setCursor(0,  10); oled.print(line1);
  oled.setCursor(0,  20); oled.print(line2);

  oled.display();
}

void refreshOLED() {
  // Tampilkan: mode, sensor depan, encoder, yaw
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Baris 0: Command + Depan
  oled.setCursor(0, 0);
  oled.printf("%-8s F:%.0fcm", currentCommand.c_str(), ultra_front);

  // Baris 1: Encoder kiri dan kanan
  oled.setCursor(0, 10);
  oled.printf("L:%-5ld R:%-5ld", enc_left, enc_right);

  // Baris 2: Yaw dalam derajat
  oled.setCursor(0, 20);
  float yaw_deg = yaw_rad * 180.0f / PI;
  oled.printf("Yaw:%+.1f L:%.0f R:%.0f", yaw_deg, ultra_left, ultra_right);

  oled.display();
}
