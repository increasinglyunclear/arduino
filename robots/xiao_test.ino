/*
 * XIAO ESP32-S3 Plus — Hardware Test
 *
 * Runs through each subsystem and reports results over Serial (115200).
 * Tests: Serial, LED, WiFi, BLE, GPIO digital output, ADC, I2C scan.
 *
 * Board: XIAO_ESP32S3 PLUS in Arduino IDE
 * Kevin Walker 17 MAR 2026
 */

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>

#define LED_PIN 21

const int testPins[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10 };
const char* pinNames[] = {
  "D0 (GPIO1)", "D1 (GPIO2)", "D2 (GPIO3)", "D3 (GPIO4)",
  "D4 (GPIO5/SDA)", "D5 (GPIO6/SCL)", "D6 (GPIO43/TX)", "D7 (GPIO44/RX)",
  "D8 (GPIO7/SCK)", "D9 (GPIO8/MISO)", "D10 (GPIO9/MOSI)"
};
const int NUM_PINS = sizeof(testPins) / sizeof(testPins[0]);

const int analogPins[] = { A0, A1, A2, A3 };
const char* analogNames[] = { "A0 (D0)", "A1 (D1)", "A2 (D2)", "A3 (D3)" };
const int NUM_ANALOG = sizeof(analogPins) / sizeof(analogPins[0]);

void printHeader(const char* title) {
  Serial.println();
  Serial.println("════════════════════════════════════════");
  Serial.print("  ");
  Serial.println(title);
  Serial.println("════════════════════════════════════════");
}

// ── Test 1: Serial ──────────────────────────────────────────
void testSerial() {
  printHeader("TEST 1: Serial Communication");
  Serial.println("  If you can read this, Serial is OK.");
  Serial.print("  CPU freq: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  Serial.print("  Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
  Serial.println(" MB");
  Serial.print("  Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("  PSRAM size: ");
  Serial.print(ESP.getPsramSize() / 1024);
  Serial.println(" KB");
  Serial.println("  RESULT: PASS");
}

// ── Test 2: Built-in LED ────────────────────────────────────
void testLED() {
  printHeader("TEST 2: Built-in LED (GPIO 21)");
  pinMode(LED_PIN, OUTPUT);
  Serial.println("  Blinking 3 times...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
  Serial.println("  Did you see the LED blink? If yes, PASS.");
}

// ── Test 3: WiFi ────────────────────────────────────────────
void testWiFi() {
  printHeader("TEST 3: WiFi Scan");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("  Scanning...");
  int n = WiFi.scanNetworks();
  if (n < 0) {
    Serial.println("  ERROR: WiFi scan failed!");
    Serial.println("  RESULT: FAIL");
  } else if (n == 0) {
    Serial.println("  No networks found (might be OK in shielded area).");
    Serial.println("  RESULT: WARN");
  } else {
    Serial.print("  Found ");
    Serial.print(n);
    Serial.println(" network(s):");
    for (int i = 0; i < n && i < 10; i++) {
      Serial.print("    ");
      Serial.print(i + 1);
      Serial.print(". ");
      Serial.print(WiFi.SSID(i));
      Serial.print("  (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(" dBm)");
    }
    if (n > 10) {
      Serial.print("    ... and ");
      Serial.print(n - 10);
      Serial.println(" more");
    }
    Serial.println("  RESULT: PASS");
  }
  WiFi.mode(WIFI_OFF);
}

// ── Test 4: BLE ─────────────────────────────────────────────
void testBLE() {
  printHeader("TEST 4: BLE Advertise");
  Serial.println("  Initializing BLE...");

  BLEDevice::init("XIAO-Test");
  BLEServer* pServer = BLEDevice::createServer();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.println("  Advertising as 'XIAO-Test' for 3 seconds...");
  Serial.println("  (check with a BLE scanner app on your phone)");
  delay(3000);

  pAdv->stop();
  BLEDevice::deinit(false);

  Serial.println("  BLE initialized and advertised successfully.");
  Serial.println("  RESULT: PASS");
}

// ── Test 5: GPIO Digital Output ─────────────────────────────
void testGPIO() {
  printHeader("TEST 5: GPIO Digital Output Sweep");
  Serial.println("  Each pin goes HIGH for 5 seconds.");
  Serial.println("  Check with LED + 220-330 ohm resistor to GND,");
  Serial.println("  or multimeter (~3.3V).");
  Serial.println("  Press any key in Serial Monitor to skip ahead.");
  Serial.println();

  for (int i = 0; i < NUM_PINS; i++) {
    // First make sure all pins are LOW
    for (int j = 0; j < NUM_PINS; j++) {
      pinMode(testPins[j], OUTPUT);
      digitalWrite(testPins[j], LOW);
    }

    Serial.print("  >> ");
    Serial.print(pinNames[i]);
    Serial.print(" = HIGH");

    pinMode(testPins[i], OUTPUT);
    digitalWrite(testPins[i], HIGH);

    unsigned long t0 = millis();
    bool skipped = false;
    while (millis() - t0 < 5000) {
      if (Serial.available()) {
        Serial.read();
        skipped = true;
        break;
      }
      delay(50);
    }

    digitalWrite(testPins[i], LOW);
    pinMode(testPins[i], INPUT);
    Serial.println(skipped ? "  (skipped)" : "");
  }

  // Reset all to input
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(testPins[i], INPUT);
  }

  Serial.println("  GPIO sweep complete.");
  Serial.println("  RESULT: manual verification needed");
}

// ── Test 6: Analog Read ─────────────────────────────────────
void testAnalog() {
  printHeader("TEST 6: Analog Read (ADC)");
  Serial.println("  Reading A0-A3. Touch a pin to see values change.");
  Serial.println("  Taking 3 readings per pin...");
  Serial.println();

  for (int i = 0; i < NUM_ANALOG; i++) {
    Serial.print("  ");
    Serial.print(analogNames[i]);
    Serial.print(": ");
    for (int r = 0; r < 3; r++) {
      int val = analogRead(analogPins[i]);
      Serial.print(val);
      if (r < 2) Serial.print(", ");
      delay(100);
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("  Values should be 0-4095. Floating pins give random noise.");
  Serial.println("  RESULT: PASS (if values printed above)");
}

// ── Test 7: I2C Scan ────────────────────────────────────────
void testI2C() {
  printHeader("TEST 7: I2C Bus Scan (SDA=D4, SCL=D5)");

  Wire.begin(D4, D5);
  delay(100);

  int found = 0;
  Serial.println("  Scanning addresses 0x01 - 0x7F...");

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);

      if (addr == 0x28 || addr == 0x29)      Serial.print("  (BNO055)");
      else if (addr == 0x68 || addr == 0x69) Serial.print("  (MPU6050)");
      else if (addr == 0x76 || addr == 0x77) Serial.print("  (BMP280/BME280)");
      else if (addr == 0x3C || addr == 0x3D) Serial.print("  (OLED SSD1306)");

      Serial.println();
      found++;
    }
  }

  Wire.end();

  if (found == 0) {
    Serial.println("  No I2C devices found.");
    Serial.println("  (This is normal if nothing is connected.)");
  } else {
    Serial.print("  ");
    Serial.print(found);
    Serial.println(" device(s) found.");
  }
  Serial.println("  RESULT: PASS (bus operational)");
}

// ── Setup & Loop ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║  XIAO ESP32-S3 Plus — Hardware Test      ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Open Serial Monitor at 115200 baud.");
  Serial.println("Tests will run automatically.");
  Serial.println("During GPIO test, press any key to skip a pin.");

  testSerial();
  testLED();
  testWiFi();
  testBLE();
  testGPIO();
  testAnalog();
  testI2C();

  printHeader("ALL TESTS COMPLETE");
  Serial.println("  Review results above.");
  Serial.println("  To re-run, press the RESET button.");
}

void loop() {
  delay(10000);
}
