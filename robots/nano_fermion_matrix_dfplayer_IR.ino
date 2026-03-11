/*
 * Fermion HR8833 + DFPlayer + LED Matrix + IR Emitter — Standalone Demo
 * Arduino Nano 33 BLE Sense Rev 2 + 3.7V LiPo
 * Kevin Walker 11 Mar 2026
 *
 * Motor tests with sequential audio, visual feedback, and continuous
 * IR emission for proximity detection by the other robot.
 *
 * IR emitter is always ON so the receiver (separate sketch) can measure distance
 * at any time during the demo sequence.
 *
 * Pin usage:
 *   D0 (TX) → DFPlayer RX         A4 (SDA) → LED matrix SDA
 *   D1 (RX) → DFPlayer TX         A5 (SCL) → LED matrix SCL
 *   D2      → IR transistor base   DFPlayer VCC → Battery +
 *   D6      → Fermion IB2 (M2B)   Fermion VCC → 3V3
 *   D7      → Fermion IB1 (M2A)   Fermion VM → Battery +
 *   D8      → Fermion IA2 (M1B)   Matrix VCC → 3V3
 *   D9      → Fermion IA1 (M1A)   All GND → common
 *   Arduino VIN → Battery +
 *
 * IR emitter circuit (powered from battery for max brightness):
 *   D2 → 1kΩ (200+800) → BC547 base
 *   BC547 emitter → GND
 *   BC547 collector → IR LED cathode (short leg)
 *   IR LED anode (long leg) → 2x 100Ω parallel (=50Ω) → Battery +
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <DFRobotDFPlayerMini.h>

DFRobotDFPlayerMini myDFPlayer;
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

const int M1A = 9;
const int M1B = 8;
const int M2A = 7;
const int M2B = 6;

const int IR_PIN = 2;

const int NUM_TRACKS = 3;
int currentTrack = 1;

void setup() {
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  stopAll();

  // IR emitter ON immediately
  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, HIGH);

  // Startup blinks
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  matrix.begin(0x70);
  matrix.setBrightness(6);
  matrix.setRotation(0);
  matrix.clear();
  matrix.writeDisplay();

  Serial1.begin(9600);
  delay(1500);

  if (!myDFPlayer.begin(Serial1)) {
    matrix.clear();
    matrix.setCursor(2, 0);
    matrix.setTextColor(LED_RED);
    matrix.print("X");
    matrix.writeDisplay();

    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  myDFPlayer.volume(25);
  delay(200);

  matrix.clear();
  matrix.setCursor(0, 0);
  matrix.setTextColor(LED_GREEN);
  matrix.print("OK");
  matrix.writeDisplay();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  delay(2000);
}

// ── Motor helpers ────────────────────────────────────────────

void stopAll() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
}

void motor1Forward()  { digitalWrite(M1A, HIGH); digitalWrite(M1B, LOW); }
void motor1Reverse()  { digitalWrite(M1A, LOW);  digitalWrite(M1B, HIGH); }
void motor2Forward()  { digitalWrite(M2A, HIGH); digitalWrite(M2B, LOW); }
void motor2Reverse()  { digitalWrite(M2A, LOW);  digitalWrite(M2B, HIGH); }
void bothForward()    { motor1Forward(); motor2Forward(); }
void bothReverse()    { motor1Reverse(); motor2Reverse(); }

// ── Display helpers ──────────────────────────────────────────

void displayNumber(int num, uint8_t color) {
  matrix.clear();
  matrix.setTextSize(1);
  matrix.setTextColor(color);
  matrix.setCursor(num < 10 ? 2 : 0, 0);
  matrix.print(num);
  matrix.writeDisplay();
}

void displayForwardArrow(uint8_t color) {
  matrix.clear();
  matrix.drawPixel(2, 3, color);
  matrix.drawPixel(3, 2, color);
  matrix.drawPixel(3, 3, color);
  matrix.drawPixel(3, 4, color);
  matrix.drawPixel(4, 1, color);
  matrix.drawPixel(4, 2, color);
  matrix.drawPixel(4, 3, color);
  matrix.drawPixel(4, 4, color);
  matrix.drawPixel(4, 5, color);
  matrix.drawPixel(5, 3, color);
  matrix.writeDisplay();
}

void displayReverseArrow(uint8_t color) {
  matrix.clear();
  matrix.drawPixel(5, 3, color);
  matrix.drawPixel(4, 2, color);
  matrix.drawPixel(4, 3, color);
  matrix.drawPixel(4, 4, color);
  matrix.drawPixel(3, 1, color);
  matrix.drawPixel(3, 2, color);
  matrix.drawPixel(3, 3, color);
  matrix.drawPixel(3, 4, color);
  matrix.drawPixel(3, 5, color);
  matrix.drawPixel(2, 3, color);
  matrix.writeDisplay();
}

void displayRestState() {
  matrix.clear();
  matrix.setCursor(2, 0);
  matrix.setTextColor(LED_RED);
  matrix.print("0");
  matrix.writeDisplay();
}

// ── Audio ────────────────────────────────────────────────────

int playNextTrack() {
  int track = currentTrack;
  myDFPlayer.play(track);
  currentTrack++;
  if (currentTrack > NUM_TRACKS) currentTrack = 1;
  return track;
}

// ── Test sequences ───────────────────────────────────────────

void testMotor1() {
  digitalWrite(LED_BUILTIN, HIGH);

  int track = playNextTrack();
  delay(300);
  displayNumber(track, LED_GREEN);
  delay(500);

  displayForwardArrow(LED_GREEN);
  motor1Forward();
  delay(2500);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  displayReverseArrow(LED_YELLOW);
  motor1Reverse();
  delay(2500);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);
}

void testMotor2() {
  digitalWrite(LED_BUILTIN, HIGH);

  int track = playNextTrack();
  delay(300);
  displayNumber(track, LED_GREEN);
  delay(500);

  displayForwardArrow(LED_GREEN);
  motor2Forward();
  delay(2500);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  displayReverseArrow(LED_YELLOW);
  motor2Reverse();
  delay(2500);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);
}

void testBothMotors() {
  digitalWrite(LED_BUILTIN, HIGH);

  int track = playNextTrack();
  delay(300);
  displayNumber(track, LED_GREEN);
  delay(500);

  displayForwardArrow(LED_GREEN);
  bothForward();
  delay(3000);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  displayReverseArrow(LED_YELLOW);
  bothReverse();
  delay(3000);
  stopAll();

  digitalWrite(LED_BUILTIN, LOW);
  displayRestState();
  delay(1000);
}

// ── Main loop ────────────────────────────────────────────────

void loop() {
  testMotor1();
  delay(1000);

  testMotor2();
  delay(1000);

  testBothMotors();
  delay(1000);

  for (int i = 0; i < 3; i++) {
    matrix.clear();
    matrix.writeDisplay();
    delay(300);

    matrix.clear();
    matrix.setCursor(0, 0);
    matrix.setTextColor(LED_GREEN);
    matrix.print("OK");
    matrix.writeDisplay();
    delay(300);
  }

  delay(2000);
}
