/*
 * XIAO ESP32-S3 Plus + BNO055 — Heading Test
 *
 * Wiring:
 *   BNO055 VCC → XIAO 3V3
 *   BNO055 GND → XIAO GND
 *   BNO055 SDA → XIAO D3 (IO4)
 *   BNO055 SCL → XIAO D5 (IO6)
 *
 * Auto-calibrates: leave still for ~3 seconds after power-on.
 * Heading is usable as soon as gyro reaches level 2+.
 * Kevin Walker 17 MAR 2026
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define I2C_SDA D3  // IO4
#define I2C_SCL D5  // IO6

Adafruit_BNO055* bno = nullptr;
bool ready = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("BNO055 Heading Test");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(1000);

  uint8_t addrs[] = { 0x28, 0x29 };
  for (int i = 0; i < 2; i++) {
    bno = new Adafruit_BNO055(55, addrs[i], &Wire);
    if (bno->begin()) {
      bno->setExtCrystalUse(true);
      Serial.printf("BNO055 found at 0x%02X\n", addrs[i]);
      Serial.println("Leave still for 3 seconds to calibrate gyro...");
      return;
    }
    delete bno;
    bno = nullptr;
  }

  Serial.println("ERROR: BNO055 not detected! Check wiring.");
}

void loop() {
  if (!bno) { delay(5000); return; }

  sensors_event_t event;
  bno->getEvent(&event);
  float heading = event.orientation.x;

  uint8_t sys, gyro, accel, mag;
  bno->getCalibration(&sys, &gyro, &accel, &mag);

  if (!ready && gyro >= 2) {
    ready = true;
    Serial.println("Gyro calibrated — heading is live!\n");
  }

  if (ready) {
    Serial.printf("Heading: %6.1f°    [G:%d M:%d]\n", heading, gyro, mag);
  } else {
    Serial.printf("Waiting... G:%d (need 2+)  keep still\n", gyro);
  }

  delay(300);
}
