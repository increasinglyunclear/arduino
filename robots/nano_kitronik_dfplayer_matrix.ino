/*
 * Arduino Nano 33 BLE Sense Rev 1 + Kitronik 5108 + DFPlayer Mini + Adafruit 8x8 Bi-color LED matrix
 * Kevin Walker 22 Feb 2026
 *
 * Powered by 3.7V / 2A LiPo. 
 *
 * Hardware Connections:
 *
 *   Nano 33 BLE          Kitronik 5108
 *   -----------          -------------
 *   D2  ──────────────── AIN1
 *   D3  ──────────────── AIN2
 *   D4  ──────────────── BIN1
 *   D5  ──────────────── BIN2
 *   D6  ──────────────── nSLEEP
 *   GND ──────────────── GND
 *                        Vcc ── LiPo 3.7V positive
 *                        GND ── LiPo negative
 *
 *   Nano 33 BLE          DFPlayer Mini
 *   -----------          -------------
 *   D0 (TX) ──────────── RX   (3.3V logic, no resistor needed)
 *   D1 (RX) ──────────── TX
 *   GND     ──────────── GND
 *                        VCC ── LiPo 3.7V positive (3.2-5.0V accepted)
 *                        SPK1 ── Speaker +
 *                        SPK2 ── Speaker -
 *
 *   Nano 33 BLE          Adafruit 8x8 Bicolor Matrix (I2C, addr 0x70)
 *   -----------          ---------------
 *   A4 (SDA) ──────────── SDA
 *   A5 (SCL) ──────────── SCL
 *   GND      ──────────── GND
 *                         VCC ── LiPo 3.7V positive
 *
 *   Power the Nano via LiPo positive to the VIN pin, negative to common GND.
 *
 * SD Card: FAT32 formatted, files named 0001.mp3, 0002.mp3, 0003.mp3 in root.
 *
 * Required Libraries:
 *   DFRobotDFPlayerMini, Adafruit_GFX, Adafruit_LEDBackpack
 *
 * NOTE: Do NOT use "AIN1" / "AIN2" as variable names —
 *       they conflict with nRF52840 PinName enum values.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <DFRobotDFPlayerMini.h>

// Motor pins (prefixed MOT_ to avoid nRF52840 PinName conflicts)
#define MOT_AIN1 2
#define MOT_AIN2 3
#define MOT_BIN1 4
#define MOT_BIN2 5
#define MOT_NSLEEP 6

#define LED_PIN LED_BUILTIN

const int LINE_DELAY = 120;

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;
int currentTrack = 1;

void setup() {
  // Motor pins LOW immediately to prevent spurious spinning during boot
  pinMode(MOT_AIN1, OUTPUT);  digitalWrite(MOT_AIN1, LOW);
  pinMode(MOT_AIN2, OUTPUT);  digitalWrite(MOT_AIN2, LOW);
  pinMode(MOT_BIN1, OUTPUT);  digitalWrite(MOT_BIN1, LOW);
  pinMode(MOT_BIN2, OUTPUT);  digitalWrite(MOT_BIN2, LOW);

  // Keep motor driver asleep until ready
  pinMode(MOT_NSLEEP, OUTPUT);
  digitalWrite(MOT_NSLEEP, LOW);

  // LED for status
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // OFF (active LOW)

  // Init LED matrix
  matrix.begin(0x70);
  matrix.setBrightness(15);
  matrix.fillScreen(LED_RED);
  matrix.writeDisplay();

  // Init DFPlayer on hardware Serial1
  Serial1.begin(9600);
  delay(500);

  if (dfPlayer.begin(Serial1)) {
    dfPlayerReady = true;
    dfPlayer.volume(20);
  }

  // Blink LED: 3 = all good, 10 rapid = DFPlayer failed
  if (dfPlayerReady) {
    blinkLED(3);
  } else {
    for (int i = 0; i < 10; i++) blinkLED(1);
  }

  // Startup delay so you can set the robot down
  delay(2000);

  // Wake motor driver
  digitalWrite(MOT_NSLEEP, HIGH);
}

void loop() {
  // ── Phase 1: Forward ── YELLOW ── Track ──
  playNextTrack();
  motorsForward();
  sweepToColor(LED_YELLOW);
  delay(1000);
  motorsStop();
  delay(500);

  // ── Phase 2: Stopped ── GREEN ──
  sweepToColor(LED_GREEN);
  delay(1000);

  // ── Phase 3: Reverse ── YELLOW ── Track ──
  playNextTrack();
  motorsReverse();
  sweepToColor(LED_YELLOW);
  delay(1000);
  motorsStop();
  delay(500);

  // ── Phase 4: Stopped ── RED ──
  sweepToColor(LED_RED);
  delay(1000);

  // ── Phase 5: Motor A only ── Track ──
  playNextTrack();
  motorForward('A');
  delay(1000);
  motorStop('A');
  delay(500);

  // ── Phase 6: Motor B only ──
  motorForward('B');
  delay(1000);
  motorStop('B');
  delay(500);

  // ── Phase 7: Both forward then brake ──
  motorsForward();
  sweepToColor(LED_GREEN);
  delay(1000);

  motorBrake('A');
  motorBrake('B');
  sweepToColor(LED_RED);
  delay(500);

  motorsStop();

  blinkLED(1);
  delay(2000);
}

// ── Motor Control ──────────────────────────────────────────────

void motorForward(char motor) {
  if (motor == 'A') {
    digitalWrite(MOT_AIN1, HIGH);
    digitalWrite(MOT_AIN2, LOW);
  } else {
    digitalWrite(MOT_BIN1, HIGH);
    digitalWrite(MOT_BIN2, LOW);
  }
}

void motorReverse(char motor) {
  if (motor == 'A') {
    digitalWrite(MOT_AIN1, LOW);
    digitalWrite(MOT_AIN2, HIGH);
  } else {
    digitalWrite(MOT_BIN1, LOW);
    digitalWrite(MOT_BIN2, HIGH);
  }
}

void motorStop(char motor) {
  if (motor == 'A') {
    digitalWrite(MOT_AIN1, LOW);
    digitalWrite(MOT_AIN2, LOW);
  } else {
    digitalWrite(MOT_BIN1, LOW);
    digitalWrite(MOT_BIN2, LOW);
  }
}

void motorBrake(char motor) {
  if (motor == 'A') {
    digitalWrite(MOT_AIN1, HIGH);
    digitalWrite(MOT_AIN2, HIGH);
  } else {
    digitalWrite(MOT_BIN1, HIGH);
    digitalWrite(MOT_BIN2, HIGH);
  }
}

void motorsForward() {
  motorForward('A');
  motorForward('B');
}

void motorsReverse() {
  motorReverse('A');
  motorReverse('B');
}

void motorsStop() {
  motorStop('A');
  motorStop('B');
}

// ── LED Matrix ─────────────────────────────────────────────────

void drawLine(int row, uint8_t color) {
  for (int col = 0; col < 8; col++) {
    matrix.drawPixel(col, row, color);
  }
  matrix.writeDisplay();
}

void sweepToColor(uint8_t newColor) {
  for (int row = 0; row < 8; row++) {
    drawLine(row, newColor);
    delay(LINE_DELAY);
  }
}

// ── DFPlayer ───────────────────────────────────────────────────

void playNextTrack() {
  if (!dfPlayerReady) return;
  dfPlayer.play(currentTrack);
  currentTrack++;
  if (currentTrack > 3) currentTrack = 1;
}

// ── Status LED ─────────────────────────────────────────────────

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, LOW);  // ON
    delay(150);
    digitalWrite(LED_PIN, HIGH); // OFF
    delay(150);
  }
}
