/*
 * Robot-A (Emitter) — MaUWB ESP32S3 (UWB Tag)
 * Drive forward with gentle gyro trim and instant distance reaction.
 * Kevin Walker 29 Mar 2026
 *
 * Sequence:
 *   1. Init all hardware (~20s — place facing Receiver during this)
 *   2. Wait for first UWB reading, record gyro heading as targetHdg
 *   3. Drive forward with proportional motor trim to hold heading
 *   4. Stop instantly when distance < 15 cm
 *   5. If smoothed distance exceeds best-seen by 15cm (3 consecutive
 *      readings): stop, turn toward targetHdg, resume. Never gives up —
 *      pauses 3s then retries with fresh baseline. Only arrival stops it.
 *
 * Wiring (MaUWB ESP32S3 AT v1.2):
 *   [Internal] GPIO16    STM32 RESET
 *   [Internal] GPIO17/18 STM32 AT UART
 *   [Internal] GPIO38/39 OLED I2C (Wire bus 0)
 *   IO1       1K resistor -> DFPlayer Mini RX
 *   IO2       DFPlayer Mini TX
 *   IO5       BNO055 SDA (Wire1, second I2C bus)
 *   IO6       BNO055 SCL (Wire1, second I2C bus)
 *   IO7       Motor 1 signal (DFRobot 75:1 servo-style)
 *   IO8       Motor 2 signal (DFRobot 75:1 servo-style)
 *   IO9       MAX7219 DIN
 *   IO10      MAX7219 CLK
 *   IO11      MAX7219 CS
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <DFRobotDFPlayerMini.h>
#include "LedControl.h"

#define SERIAL_LOG Serial
HardwareSerial SERIAL_AT(2);

#define RESET_PIN  16
#define IO_RXD2    18
#define IO_TXD2    17
#define I2C_SDA    39
#define I2C_SCL    38

#define DFPLAYER_TX 1
#define DFPLAYER_RX 2
#define BNO_SDA     5
#define BNO_SCL     6
#define MOTOR1_PIN  7
#define MOTOR2_PIN  8
#define MATRIX_DIN  9
#define MATRIX_CLK  10
#define MATRIX_CS   11

const int MOTOR_STOP     = 1500;
const int MOTOR_FWD      = 2000;
const int MOTOR_REV      = 1000;
const int MOTOR_HALF_FWD = 1750;
const int MOTOR_HALF_REV = 1250;

int motor1_us = 0;
int motor2_us = 0;

// BNO055
TwoWire bnoWire(1);
Adafruit_BNO055* bno = nullptr;
bool imuOK = false;

// Peripherals
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;
LedControl* lc = nullptr;

const uint8_t arrows[8][8] = {
  {0x10, 0x28, 0x44, 0x10, 0x10, 0x10, 0x10, 0x00},
  {0x02, 0x02, 0x06, 0x08, 0x10, 0x20, 0x40, 0x00},
  {0x00, 0x04, 0x02, 0xFF, 0x02, 0x04, 0x00, 0x00},
  {0x00, 0x40, 0x20, 0x10, 0x08, 0x06, 0x02, 0x02},
  {0x00, 0x10, 0x10, 0x10, 0x10, 0x44, 0x28, 0x10},
  {0x00, 0x02, 0x04, 0x08, 0x10, 0x60, 0x40, 0x40},
  {0x00, 0x20, 0x40, 0xFF, 0x40, 0x20, 0x00, 0x00},
  {0x40, 0x40, 0x60, 0x10, 0x08, 0x04, 0x02, 0x00},
};

// State machine
enum Phase { PH_WAIT_UWB, PH_DRIVE, PH_CORRECT, PH_PAUSE, PH_ARRIVED };
Phase phase = PH_WAIT_UWB;

float targetHdg = 0;
float minDist = 9999;
int correctionsDone = 0;
int overshootCount = 0;
unsigned long driveResumeMs = 0;
unsigned long pauseStartMs = 0;

// Smoothed UWB distance (median-of-5 filter)
const int DBUF_SZ = 5;
float distBuf[DBUF_SZ];
int distBufIdx = 0;
int distBufN = 0;
float smoothDist = -1;

// UWB
String response = "";
float lastDist = -1;
unsigned long lastUpdate = 0;

// OLED rate limit
unsigned long lastOledRefresh = 0;

// ── Low-level helpers ────────────────────────────────────

String sendAT(const String& cmd, unsigned long timeout) {
  SERIAL_LOG.println(cmd);
  SERIAL_AT.println(cmd);
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (SERIAL_AT.available()) resp += (char)SERIAL_AT.read();
  }
  SERIAL_LOG.println(resp);
  return resp;
}

void oledShow(const String& line1, const String& line2 = "",
              const String& line3 = "") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(line1);
  if (line2.length()) { display.setCursor(0, 20); display.setTextSize(2); display.println(line2); }
  if (line3.length()) { display.setCursor(0, 48); display.setTextSize(1); display.println(line3); }
  display.display();
}

void parseRange(const String& line) {
  int rs = line.indexOf("range:("), as = line.indexOf("ancid:(");
  if (rs < 0 || as < 0) return;
  rs += 7; as += 7;
  int re = line.indexOf(')', rs), ae = line.indexOf(')', as);
  if (re < 0 || ae < 0) return;

  String rangeStr = line.substring(rs, re);
  String ancidStr = line.substring(as, ae);
  int ranges[8] = {0}, ancids[8] = {-1};

  int idx = 0, pos = 0;
  while (idx < 8 && pos <= (int)rangeStr.length()) {
    int c = rangeStr.indexOf(',', pos); if (c < 0) c = rangeStr.length();
    ranges[idx++] = rangeStr.substring(pos, c).toInt(); pos = c + 1;
  }
  idx = 0; pos = 0;
  while (idx < 8 && pos <= (int)ancidStr.length()) {
    int c = ancidStr.indexOf(',', pos); if (c < 0) c = ancidStr.length();
    ancids[idx++] = ancidStr.substring(pos, c).toInt(); pos = c + 1;
  }

  for (int i = 0; i < 8; i++) {
    if (ancids[i] < 0) continue;
    lastDist = ranges[i];
    lastUpdate = millis();
    feedSmooth(lastDist);
    SERIAL_LOG.printf("  UWB: %d cm (smooth=%.0f)\n", ranges[i], smoothDist);
  }
}

void feedSmooth(float raw) {
  distBuf[distBufIdx] = raw;
  distBufIdx = (distBufIdx + 1) % DBUF_SZ;
  if (distBufN < DBUF_SZ) distBufN++;

  float tmp[DBUF_SZ];
  for (int i = 0; i < distBufN; i++) tmp[i] = distBuf[i];
  for (int i = 0; i < distBufN - 1; i++)
    for (int j = i + 1; j < distBufN; j++)
      if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  smoothDist = tmp[distBufN / 2];
}

// ── Motor helpers ────────────────────────────────────────

void updateServos() {
  static unsigned long lastPulse = 0;
  if (micros() - lastPulse < 20000) return;
  lastPulse = micros();
  if (motor1_us > 0) { digitalWrite(MOTOR1_PIN, HIGH); delayMicroseconds(motor1_us); digitalWrite(MOTOR1_PIN, LOW); }
  if (motor2_us > 0) { digitalWrite(MOTOR2_PIN, HIGH); delayMicroseconds(motor2_us); digitalWrite(MOTOR2_PIN, LOW); }
}

void stopAll()       { motor1_us = MOTOR_STOP; motor2_us = MOTOR_STOP; }
void motorsOff()     { motor1_us = 0;          motor2_us = 0; }
void slowTurnLeft()  { motor1_us = MOTOR_HALF_REV; motor2_us = MOTOR_HALF_REV; }
void slowTurnRight() { motor1_us = MOTOR_HALF_FWD; motor2_us = MOTOR_HALF_FWD; }

void forwardWithTrim(float drift) {
  int base1 = MOTOR_REV;   // 1000
  int base2 = MOTOR_FWD;   // 2000
  // drift > 0 means drifted right → nudge left by slowing motor2
  // drift < 0 means drifted left  → nudge right by slowing motor1
  int trim = constrain((int)(drift * 4), -150, 150);
  motor1_us = constrain(base1 + trim, 850, 1200);
  motor2_us = constrain(base2 - trim, 1800, 2150);
}

// ── IMU helpers ──────────────────────────────────────────

float readHeading() {
  if (!imuOK) return 0;
  sensors_event_t e;
  bno->getEvent(&e);
  return e.orientation.x;
}

float headingDiff(float from, float to) {
  float d = to - from;
  while (d >  180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// ── Matrix helpers ───────────────────────────────────────

uint8_t reverseBits(uint8_t b) {
  b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
  b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
  b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
  return b;
}

void showArrow(int dir) {
  if (!lc || dir < 0 || dir > 7) return;
  for (int r = 0; r < 8; r++)
    lc->setRow(0, r, reverseBits(arrows[dir][7 - r]));
}

void matrixFull() {
  if (!lc) return;
  for (int r = 0; r < 8; r++) lc->setRow(0, r, 0xFF);
}

void matrixOff() {
  if (!lc) return;
  for (int r = 0; r < 8; r++) lc->setRow(0, r, 0x00);
}

// ── Arduino entry points ─────────────────────────────────

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  SERIAL_LOG.begin(115200);
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOK = true;
    display.clearDisplay();
  }

  oledShow("Robot-A Emitter", "Init...");
  SERIAL_LOG.println("=== Robot-A (Emitter) Trim Drive ===");

  sendAT("AT?", 2000);
  sendAT("AT+RESTORE", 5000);
  sendAT("AT+SETCFG=0,0,1,1", 2000);
  sendAT("AT+SETCAP=1,10,1", 2000);
  sendAT("AT+SETRPT=1", 2000);
  sendAT("AT+SETANT=16528", 2000);
  sendAT("AT+SAVE", 2000);
  sendAT("AT+RESTART", 2000);

  Serial1.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(500);
  if (dfPlayer.begin(Serial1)) {
    dfPlayerOK = true;
    dfPlayer.volume(30);
    SERIAL_LOG.println("DFPlayer: OK");
  } else {
    SERIAL_LOG.println("DFPlayer: NOT FOUND");
  }

  pinMode(MATRIX_DIN, OUTPUT);
  pinMode(MATRIX_CLK, OUTPUT);
  pinMode(MATRIX_CS, OUTPUT);
  lc = new LedControl(MATRIX_DIN, MATRIX_CLK, MATRIX_CS, 1);
  lc->shutdown(0, false);
  lc->setIntensity(0, 6);
  matrixOff();
  SERIAL_LOG.println("MAX7219: OK");

  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  SERIAL_LOG.println("Motors: OK");

  bnoWire.begin(BNO_SDA, BNO_SCL);
  bnoWire.setClock(100000);
  delay(1000);
  uint8_t addrs[] = { 0x28, 0x29 };
  for (int i = 0; i < 2; i++) {
    bno = new Adafruit_BNO055(55, addrs[i], &bnoWire);
    if (bno->begin()) {
      bno->setMode(OPERATION_MODE_IMUPLUS);
      delay(100);
      imuOK = true;
      SERIAL_LOG.printf("BNO055: OK at 0x%02X (IMUPLUS)\n", addrs[i]);
      break;
    }
    delete bno;
    bno = nullptr;
  }
  if (!imuOK) SERIAL_LOG.println("BNO055: NOT FOUND");

  phase = PH_WAIT_UWB;
  oledShow("READY", "Face Rx", "Waiting for UWB...");
  SERIAL_LOG.println("Place facing Receiver. Will drive on first UWB reading.");
}

void loop() {
  updateServos();

  // Read one UWB line per loop
  while (SERIAL_AT.available()) {
    char c = SERIAL_AT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (response.length() > 0) {
        if (response.startsWith("AT+RANGE")) {
          parseRange(response);
          response = "";
          break;
        }
        response = "";
      }
    } else {
      response += c;
    }
  }

  // ── Immediate arrival check after every UWB reading ──
  if (phase == PH_DRIVE && lastDist >= 0 && lastDist < 15.0) {
    stopAll();
    phase = PH_ARRIVED;
    matrixFull();
    SERIAL_LOG.printf("ARRIVED at %.0f cm (min was %.0f)\n", lastDist, minDist);
    oledShow("ARRIVED", String((int)lastDist) + "cm", "min=" + String((int)minDist));
    return;
  }

  // ── Phase: wait for first UWB reading, then go ──
  if (phase == PH_WAIT_UWB) {
    if (smoothDist > 0) {
      targetHdg = readHeading();
      minDist = smoothDist;
      correctionsDone = 0;
      overshootCount = 0;
      driveResumeMs = millis();
      phase = PH_DRIVE;
      showArrow(0);
      SERIAL_LOG.printf("GO — heading=%.1f dist=%.0f cm\n", targetHdg, smoothDist);
    }
    return;
  }

  // ── Phase: drive forward with gyro trim ──
  if (phase == PH_DRIVE) {
    float drift = imuOK ? headingDiff(targetHdg, readHeading()) : 0;

    if (imuOK) {
      forwardWithTrim(drift);
    } else {
      motor1_us = MOTOR_REV;
      motor2_us = MOTOR_FWD;
    }

    // Track closest point using smoothed distance
    if (smoothDist > 0 && smoothDist < minDist) {
      minDist = smoothDist;
      overshootCount = 0;
    }

    // Overshoot: 3 consecutive smoothed readings past minDist+15, with 2s grace
    bool graceOver = (millis() - driveResumeMs > 2000);
    if (smoothDist > 0 && smoothDist > minDist + 15.0 && graceOver) {
      overshootCount++;
      SERIAL_LOG.printf("OVERSHOOT? %d/3: smooth=%.0f min=%.0f\n",
        overshootCount, smoothDist, minDist);
    } else if (smoothDist > 0 && smoothDist <= minDist + 15.0) {
      overshootCount = 0;
    }

    if (overshootCount >= 3) {
      stopAll();
      overshootCount = 0;
      correctionsDone++;
      if (imuOK) {
        phase = PH_CORRECT;
        float d = headingDiff(targetHdg, readHeading());
        SERIAL_LOG.printf("OVERSHOOT #%d: smooth=%.0f min=%.0f drift=%.1f — correcting\n",
          correctionsDone, smoothDist, minDist, d);
        if (d > 0) slowTurnLeft(); else slowTurnRight();
      } else {
        phase = PH_PAUSE;
        pauseStartMs = millis();
        SERIAL_LOG.printf("OVERSHOOT #%d: no IMU — pausing then retry\n", correctionsDone);
      }
      return;
    }

    // Matrix arrow shows trim direction
    if (lc) {
      int dir;
      if (drift > 20)       dir = 6;
      else if (drift > 5)   dir = 7;
      else if (drift < -20) dir = 2;
      else if (drift < -5)  dir = 1;
      else                   dir = 0;
      static int lastDir = -1;
      if (dir != lastDir) { lastDir = dir; showArrow(dir); }
    }

    // OLED
    if (millis() - lastOledRefresh >= 250) {
      lastOledRefresh = millis();
      String dist = (smoothDist > 0) ? String((int)smoothDist) + "cm" : "--";
      String info = "d=" + String((int)drift) + " min=" + String((int)minDist) + " #" + String(correctionsDone);
      oledShow("DRIVE", dist, info);
      SERIAL_LOG.printf("DRIVE: raw=%.0f smooth=%.0f min=%.0f drift=%.1f m1=%d m2=%d\n",
        lastDist, smoothDist, minDist, drift, motor1_us, motor2_us);
    }
    return;
  }

  // ── Phase: correct — turn back toward targetHdg, then resume ──
  if (phase == PH_CORRECT) {
    float drift = headingDiff(targetHdg, readHeading());

    if (fabs(drift) < 5.0) {
      stopAll();
      correctionsDone++;
      minDist = (smoothDist > 0) ? smoothDist : 9999;
      overshootCount = 0;
      driveResumeMs = millis();
      phase = PH_DRIVE;
      SERIAL_LOG.printf("CORRECT done #%d — heading=%.1f, resuming\n",
        correctionsDone, readHeading());
      return;
    }

    if (drift > 0) slowTurnLeft(); else slowTurnRight();

    if (millis() - lastOledRefresh >= 250) {
      lastOledRefresh = millis();
      String dist = (lastDist >= 0) ? String((int)lastDist) + "cm" : "--";
      oledShow("CORRECT", dist, "drift=" + String((int)drift));
    }
    return;
  }

  // ── Phase: arrived ──
  if (phase == PH_ARRIVED) {
    stopAll();
    if (millis() - lastOledRefresh >= 500) {
      lastOledRefresh = millis();
      String dist = (lastDist >= 0) ? String((int)lastDist) + "cm" : "--";
      oledShow("ARRIVED", dist, "min=" + String((int)minDist));
    }
  }

  // ── Phase: pause — wait 3s then resume driving with fresh minDist ──
  if (phase == PH_PAUSE) {
    stopAll();
    unsigned long elapsed = millis() - pauseStartMs;

    if (elapsed >= 3000) {
      minDist = (smoothDist > 0) ? smoothDist : 9999;
      overshootCount = 0;
      driveResumeMs = millis();
      phase = PH_DRIVE;
      SERIAL_LOG.printf("PAUSE done — resuming, fresh min=%.0f\n", minDist);
      return;
    }

    if (millis() - lastOledRefresh >= 250) {
      lastOledRefresh = millis();
      int secLeft = (int)((3000 - elapsed) / 1000) + 1;
      String dist = (smoothDist > 0) ? String((int)smoothDist) + "cm" : "--";
      oledShow("PAUSE " + String(secLeft), dist, "#" + String(correctionsDone) + " min=" + String((int)minDist));
    }
  }

  // Matrix health refresh
  if (lc) {
    static unsigned long lastMR = 0;
    if (millis() - lastMR >= 2000) { lastMR = millis(); lc->shutdown(0, false); lc->setIntensity(0, 6); }
  }
}
