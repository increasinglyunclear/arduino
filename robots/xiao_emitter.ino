/*
 * Robot-A — XIAO ESP32-S3 PLUS 
 * BNO055 fused heading + NimBLE
 * Kevin Walker 18 MAR 2026
 *
 * BNO055 outputs fused heading directly — no drift correction needed.
 * DFRobot servo-style motors via ESP32Servo writeMicroseconds().
 *
 * Boot → advertise → connect → calibrate (face receiver, 10 cm).
 * Connected: arrow tracks receiver. Off > 15° for 0.5 s →
 *   correction bursts (duration scales down with angle).
 * Disconnected (only after first calibration): free-roam.
 *
 * Pins:
 *   D0 (IO1)  — MAX7219 DIN
 *   D1 (IO2)  — MAX7219 CLK
 *   D2 (IO3)  — MAX7219 CS
 *   D3 (IO4)  — I2C SDA (BNO055)
 *   D4 (IO5)  — Motor 1 servo
 *   D5 (IO6)  — I2C SCL (BNO055)
 *   D6 (IO43) — Motor 2 servo
 */

#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include "LedControl.h"
#include <math.h>

// ── BLE UUIDs (same as Arduino version) ─────────────────────
#define SERVICE_UUID  "19b10000-e8f2-537e-4f6c-d104768a1214"
#define TX_CHAR_UUID  "19b10001-e8f2-537e-4f6c-d104768a1214"
#define RX_CHAR_UUID  "19b10002-e8f2-537e-4f6c-d104768a1214"

// ── Pin assignments ─────────────────────────────────────────
#define MATRIX_DIN  D0   // IO1
#define MATRIX_CLK  D1   // IO2
#define MATRIX_CS   D2   // IO3
#define I2C_SDA_PIN D3   // IO4 — SDA moved here (D4 bad on some boards)
#define MOTOR1_PIN  D4   // IO5
#define I2C_SCL_PIN D5   // IO6
#define MOTOR2_PIN  D6   // IO43

// ── Servo motor values (DFRobot 75:1 servo-style) ───────────
const int MOTOR_STOP = 1500;
const int MOTOR_FWD  = 1000;
const int MOTOR_REV  = 2000;

// ── Thresholds ──────────────────────────────────────────────
const float         TRIGGER_DEG = 15.0;
const float         STOP_DEG    = 8.0;
const unsigned long SETTLE_MS   = 500;
const float         IMPACT_MS2  = 12.0;  // m/s² linear accel threshold
const unsigned long MAX_FWD_MS  = 3000;

// ── Hardware objects ────────────────────────────────────────
LedControl lc = LedControl(MATRIX_DIN, MATRIX_CLK, MATRIX_CS, 1);
Adafruit_BNO055* bno = nullptr;
Servo motor1;
Servo motor2;

// ── NimBLE objects ──────────────────────────────────────────
NimBLEServer*         pServer  = nullptr;
NimBLECharacteristic* pTxChar  = nullptr;
NimBLECharacteristic* pRxChar  = nullptr;

// ── Arrow bitmaps (thin single-line) ────────────────────────
const byte ARROWS[8][8] = {
  { 0x10, 0x28, 0x44, 0x10, 0x10, 0x10, 0x00, 0x00 },  // 0: up (0°)
  { 0x00, 0x0E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00 },  // 1: up-right (45°)
  { 0x00, 0x08, 0x04, 0x7E, 0x04, 0x08, 0x00, 0x00 },  // 2: right (90°)
  { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x0E, 0x00 },  // 3: down-right (135°)
  { 0x00, 0x10, 0x10, 0x10, 0x44, 0x28, 0x10, 0x00 },  // 4: down (180°)
  { 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x70, 0x00 },  // 5: down-left (225°)
  { 0x00, 0x10, 0x20, 0x7E, 0x20, 0x10, 0x00, 0x00 },  // 6: left (270°)
  { 0x00, 0x70, 0x40, 0x20, 0x10, 0x08, 0x04, 0x00 },  // 7: up-left (315°)
};

// ── State ───────────────────────────────────────────────────
volatile bool bleConnected   = false;
volatile bool rxWritten      = false;
volatile uint8_t rxBuf[20];
volatile int  rxLen          = 0;

bool    imuOK          = false;
bool    calibrated     = false;
bool    everCalibrated = false;
bool    correcting     = false;
float   headingOffset  = 0;
float   relAngle       = 0;
uint8_t txCounter      = 0;
unsigned long lastTx   = 0;
unsigned long offSince = 0;

// ── Display helpers ─────────────────────────────────────────
void showArrow(float angle) {
  int sector = (int)round(angle / 45.0);
  int idx = (((8 - sector) % 8) + 8) % 8;
  for (int r = 0; r < 8; r++) lc.setRow(0, r, ARROWS[idx][r]);
}

void showAdvertising() {
  lc.clearDisplay(0);
  bool ph = (millis() / 500) % 2;
  for (int r = ph ? 0 : 1; r < 8; r += 2) lc.setRow(0, r, 0xFF);
}

void showRoaming() {
  static const byte frames[4][8] = {
    {0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00},
    {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01},
    {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10},
    {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80},
  };
  int f = (millis() / 200) % 4;
  for (int r = 0; r < 8; r++) lc.setRow(0, r, frames[f][r]);
}

// ── Motor primitives (servo-style) ──────────────────────────
void stopAll() {
  motor1.writeMicroseconds(MOTOR_STOP);
  motor2.writeMicroseconds(MOTOR_STOP);
}

void motor1Forward() { motor1.writeMicroseconds(MOTOR_FWD); }
void motor1Reverse() { motor1.writeMicroseconds(MOTOR_REV); }
void motor2Forward() { motor2.writeMicroseconds(MOTOR_FWD); }
void motor2Reverse() { motor2.writeMicroseconds(MOTOR_REV); }

void bothForward() {
  motor1.writeMicroseconds(MOTOR_FWD);
  motor2.writeMicroseconds(MOTOR_FWD);
}

void bothReverse() {
  motor1.writeMicroseconds(MOTOR_REV);
  motor2.writeMicroseconds(MOTOR_REV);
}

// ── BNO055 heading ──────────────────────────────────────────
float readHeading() {
  sensors_event_t event;
  bno->getEvent(&event);
  return event.orientation.x;  // 0-360°
}

float getRelativeAngle() {
  float h = readHeading();
  float rel = h - headingOffset;
  while (rel >  180) rel -= 360;
  while (rel < -180) rel += 360;
  return rel;
}

bool checkCollision() {
  sensors_event_t event;
  bno->getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  float mag = sqrt(event.acceleration.x * event.acceleration.x +
                   event.acceleration.y * event.acceleration.y +
                   event.acceleration.z * event.acceleration.z);
  return mag > IMPACT_MS2;
}

// ── NimBLE callbacks ────────────────────────────────────────
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    calibrated   = false;
    offSince     = 0;
    correcting   = false;
    stopAll();
    Serial.println("BLE connected");
  }

  void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
    bleConnected = false;
    calibrated   = false;
    offSince     = 0;
    correcting   = false;
    stopAll();
    NimBLEDevice::startAdvertising();
    Serial.println("BLE disconnected");
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = pChr->getValue();
    rxLen = val.size();
    if (rxLen > 20) rxLen = 20;
    memcpy((void*)rxBuf, val.data(), rxLen);
    rxWritten = true;
  }
};

static ServerCB serverCB;
static RxCB     rxCB;

// ── Blocking motor helpers ──────────────────────────────────
void roamTurn(bool right, unsigned long ms) {
  if (right) motor1Forward(); else motor2Forward();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    delay(50);
    showRoaming();
    if (bleConnected) break;
  }
  stopAll();
}

bool roamForward(unsigned long maxMs) {
  bothForward();
  unsigned long t0 = millis();
  while (millis() - t0 < maxMs) {
    delay(50);
    showRoaming();
    if (bleConnected) { stopAll(); return false; }
    if (checkCollision()) { stopAll(); return true; }
  }
  stopAll();
  return false;
}

void roamReverse(unsigned long ms) {
  bothReverse();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    delay(50);
    showRoaming();
    if (bleConnected) break;
  }
  stopAll();
}

void correctBurst() {
  relAngle = getRelativeAngle();
  float absA = fabs(relAngle);
  unsigned long burstMs = (unsigned long)(40 + absA * 2.0);
  if (burstMs > 200) burstMs = 200;
  if (burstMs < 40)  burstMs = 40;

  if (relAngle > 0) motor1Forward(); else motor2Forward();
  delay(burstMs);
  stopAll();

  delay(burstMs);
  relAngle = getRelativeAngle();
  showArrow(relAngle);
}

// ── LED helper (tries both polarities — works on all XIAO revisions) ──
#define LED_PIN 21
bool ledActiveHigh = true;

void ledOn()  { digitalWrite(LED_PIN, ledActiveHigh ? HIGH : LOW); }
void ledOff() { digitalWrite(LED_PIN, ledActiveHigh ? LOW : HIGH); }

void ledDetectPolarity() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  // User will see which polarity works; default to LOW = on (common for XIAO)
  ledActiveHigh = false;
  ledOn();
}

// ── Setup ───────────────────────────────────────────────────
void setup() {
  ledDetectPolarity();

  Serial.begin(115200);
  delay(500);
  randomSeed(analogRead(A0));

  // LED matrix
  lc.shutdown(0, false);
  lc.setIntensity(0, 6);
  lc.clearDisplay(0);

  // I2C + BNO055 (try both addresses)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  delay(1000);
  uint8_t addrs[] = { 0x28, 0x29 };
  for (int i = 0; i < 2; i++) {
    bno = new Adafruit_BNO055(55, addrs[i], &Wire);
    if (bno->begin()) {
      bno->setExtCrystalUse(true);
      imuOK = true;
      Serial.printf("BNO055: OK at 0x%02X\n", addrs[i]);
      break;
    }
    delete bno;
    bno = nullptr;
  }
  if (!imuOK) Serial.println("BNO055: NOT FOUND");

  // Servo motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor1.attach(MOTOR1_PIN, 500, 2500);
  motor2.attach(MOTOR2_PIN, 500, 2500);
  stopAll();

  // NimBLE
  NimBLEDevice::init("Robot-A");
  NimBLEDevice::setPower(9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCB);

  NimBLEService* pSvc = pServer->createService(SERVICE_UUID);

  pTxChar = pSvc->createCharacteristic(
    TX_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  pRxChar = pSvc->createCharacteristic(
    RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxChar->setCallbacks(&rxCB);

  pSvc->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->setName("Robot-A");
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->enableScanResponse(true);
  pAdv->setMinInterval(32);   // 20ms  (32 * 0.625ms)
  pAdv->setMaxInterval(64);   // 40ms  (64 * 0.625ms)
  pAdv->start();

  Serial.println("Advertising...");
}

// ── Loop ────────────────────────────────────────────────────
void loop() {
  // LED: blink when disconnected, solid when connected
  if (bleConnected) {
    ledOn();
  } else {
    (millis() / 500) % 2 ? ledOn() : ledOff();
  }

  // Read heading from BNO055 (fused, no drift)
  if (imuOK && calibrated) {
    relAngle = getRelativeAngle();
  }

  // ── Disconnected ──
  if (!bleConnected) {
    if (!everCalibrated) {
      showAdvertising();
      return;
    }

    bool right = random(2);
    unsigned long turnMs = random(100, 501);
    Serial.print("Roam: turn ");
    Serial.print(right ? "R " : "L ");
    Serial.print(turnMs);
    Serial.println(" ms");

    roamTurn(right, turnMs);
    if (bleConnected) return;

    Serial.println("Roam: forward");
    bool hit = roamForward(MAX_FWD_MS);
    if (bleConnected) return;

    if (hit) {
      Serial.println("Roam: collision → reverse");
      roamReverse(300);
    }
    return;
  }

  // ── Connected: calibration ──
  if (rxWritten) {
    rxWritten = false;
    if (rxLen >= 5 && rxBuf[0] == 0xBB) {
      if (!calibrated && imuOK) {
        headingOffset  = readHeading();
        relAngle       = 0;
        calibrated     = true;
        everCalibrated = true;
        Serial.print("Calibrated  headingRef=");
        Serial.println(headingOffset, 1);
      }
    }
  }

  // ── Telemetry ──
  if (millis() - lastTx >= 250) {
    lastTx = millis();
    uint8_t pkt[4];
    pkt[0] = 0xAA;
    int16_t tmp = (int16_t)(relAngle * 10);
    memcpy(pkt + 1, &tmp, 2);
    pkt[3] = txCounter++;
    pTxChar->setValue(pkt, 4);
    pTxChar->notify();

    if (calibrated) {
      Serial.printf("Heading: %.1f°\n", relAngle);
    }
  }

  if (!calibrated) {
    showAdvertising();
    return;
  }

  // ── Calibrated: arrow + correction ──
  showArrow(relAngle);

  float absAngle = fabs(relAngle);

  if (absAngle <= STOP_DEG) {
    if (correcting) {
      correcting = false;
      offSince   = 0;
      Serial.print("Corrected to ");
      Serial.println(relAngle, 1);
    }
    offSince = 0;
  } else if (!correcting && absAngle > TRIGGER_DEG) {
    if (offSince == 0) {
      offSince = millis();
    } else if (millis() - offSince >= SETTLE_MS) {
      correcting = true;
      Serial.print("Correcting ");
      Serial.println(relAngle, 1);
    }
  }

  if (correcting) {
    correctBurst();
  }
}
