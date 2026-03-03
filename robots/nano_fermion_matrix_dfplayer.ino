// Fermion HR8833 + DFPlayer + LED Matrix - Standalone Demo
// Arduino Nano 33 BLE Sense Rev1 + 3.7V LiPo
// Motor tests with random audio and visual feedback
// Kevin Walker 02 Mar 2026

/*
 * POWER CONNECTIONS:
 * ------------------
 * 3.7V LiPo Battery:
 *   Battery + (Red)    → Arduino VIN
 *                      → Fermion VM (motor power)
 *   Battery - (Black)  → Common GND (all components)
 * 
 * Arduino Power Outputs:
 *   Arduino 3.3V       → Fermion VCC (logic power)
 *                      → LED Matrix VCC
 *                      → DFPlayer VCC
 *   Arduino GND        → Common GND
 * 
 * 
 * MOTOR DRIVER (Fermion HR8833):
 * ------------------------------
 *   Arduino 3.3V → Fermion VCC (logic)
 *   Battery +    → Fermion VM (motor power)
 *   Arduino GND  → Fermion GND
 *   Arduino D9   → Fermion M1A
 *   Arduino D7   → Fermion M1B
 *   Arduino D8   → Fermion M2A
 *   Arduino D6   → Fermion M2B
 *   
 *   Motor 1 → Fermion Motor 1 outputs
 *   Motor 2 → Fermion Motor 2 outputs
 * 
 * 
 * DFPLAYER MINI (Audio):
 * ----------------------
 *   Arduino 3.3V     → DFPlayer VCC
 *   Arduino GND      → DFPlayer GND
 *   Arduino D0 (TX)  → 1kΩ resistor → DFPlayer RX
 *   Arduino D1 (RX)  → DFPlayer TX (direct, no resistor)
 *   
 *   DFPlayer SPK_1   → Speaker +
 *   DFPlayer SPK_2   → Speaker -
 *   
 * 
 * 
 * LED MATRIX (8x8 Bicolor I2C):
 * -----------------------------
 *   Arduino 3.3V  → LED Matrix VCC
 *   Arduino GND   → LED Matrix GND
 *   Arduino SDA (A4) → LED Matrix SDA
 *   Arduino SCL (A5) → LED Matrix SCL
 *   
 * 
 * CRITICAL NOTES:
 * ---------------
 * 1. All grounds MUST be connected together (common ground)
 * 2. 1kΩ resistor on DFPlayer RX is REQUIRED (protects DFPlayer)
 * 3. Motor pin mapping: M1B=D7, M2A=D8 (swapped from standard)
 * 4. SD card must be FAT32 formatted
 * 5. Audio files must be named 0001.mp3 through 0010.mp3
 * 
 * PIN SUMMARY:
 * ------------
 * D0  - TX to DFPlayer (via 1kΩ resistor)
 * D1  - RX from DFPlayer
 * D6  - Fermion M2B
 * D7  - Fermion M1B
 * D8  - Fermion M2A
 * D9  - Fermion M1A
 * A4  - LED Matrix SDA
 * A5  - LED Matrix SCL
 * 3.3V - Power to all logic (Fermion VCC, Matrix, DFPlayer)
 * VIN - Battery + (3.7V)
 * GND - Common ground
 * 
 * ========================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <DFRobotDFPlayerMini.h>

DFRobotDFPlayerMini myDFPlayer;
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

// Motor pins (corrected)
const int M1A = 9;
const int M1B = 7;
const int M2A = 8;
const int M2B = 6;

// Built-in LED
const int LED = LED_BUILTIN;

// Number of audio files available
const int NUM_TRACKS = 10;

void setup() {
  // Configure motor pins
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // Stop all motors
  stopAll();
  
  // Startup indication (3 quick blinks)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
  
  // Initialize LED matrix
  matrix.begin(0x70);
  matrix.setBrightness(6);
  matrix.setRotation(0);
  matrix.clear();
  matrix.writeDisplay();
  
  // Initialize DFPlayer on Serial1 (D0/D1)
  Serial1.begin(9600);
  delay(1500);
  
  if (!myDFPlayer.begin(Serial1)) {
    // Failed - show X
    matrix.clear();
    matrix.setCursor(2, 0);
    matrix.setTextColor(LED_RED);
    matrix.print("X");
    matrix.writeDisplay();
    
    // Rapid blink error
    while(1) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  }
  
  // DFPlayer OK
  myDFPlayer.volume(25);
  delay(200);
  
  // Show "OK" on startup
  matrix.clear();
  matrix.setCursor(0, 0);
  matrix.setTextColor(LED_GREEN);
  matrix.print("OK");
  matrix.writeDisplay();
  
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  
  // Seed random with analog noise
  randomSeed(analogRead(A0));
  
  delay(2000);
}

void stopAll() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
}

void motor1Forward() {
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
}

void motor1Reverse() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
}

void motor2Forward() {
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
}

void motor2Reverse() {
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
}

void bothForward() {
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
}

void bothReverse() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
}

// Display functions - simple and clear
void displayNumber(int num, uint8_t color) {
  matrix.clear();
  matrix.setTextSize(1);
  matrix.setTextColor(color);
  
  if (num < 10) {
    matrix.setCursor(2, 0);
  } else {
    matrix.setCursor(0, 0);
  }
  
  matrix.print(num);
  matrix.writeDisplay();
}

void displayForwardArrow(uint8_t color) {
  matrix.clear();
  // Simple right arrow
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
  // Simple left arrow
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

// Play random track
int playRandomTrack() {
  int track = random(1, NUM_TRACKS + 1);  // Random 1-10
  myDFPlayer.play(track);
  return track;
}

// Test Motor 1 - Forward then Reverse
void testMotor1() {
  digitalWrite(LED, HIGH);
  
  // Motor 1 Forward
  int track = playRandomTrack();
  delay(300);  // Let audio start
  
  displayNumber(track, LED_GREEN);
  delay(500);
  
  displayForwardArrow(LED_GREEN);
  motor1Forward();
  delay(2500);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
  
  // Motor 1 Reverse
  digitalWrite(LED, HIGH);
  displayReverseArrow(LED_YELLOW);
  motor1Reverse();
  delay(2500);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
}

// Test Motor 2 - Forward then Reverse
void testMotor2() {
  digitalWrite(LED, HIGH);
  
  // Motor 2 Forward
  int track = playRandomTrack();
  delay(300);
  
  displayNumber(track, LED_GREEN);
  delay(500);
  
  displayForwardArrow(LED_GREEN);
  motor2Forward();
  delay(2500);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
  
  // Motor 2 Reverse
  digitalWrite(LED, HIGH);
  displayReverseArrow(LED_YELLOW);
  motor2Reverse();
  delay(2500);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
}

// Test Both Motors - Forward then Reverse
void testBothMotors() {
  digitalWrite(LED, HIGH);
  
  // Both Forward
  int track = playRandomTrack();
  delay(300);
  
  displayNumber(track, LED_GREEN);
  delay(500);
  
  displayForwardArrow(LED_GREEN);
  bothForward();
  delay(3000);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
  
  // Both Reverse
  digitalWrite(LED, HIGH);
  displayReverseArrow(LED_YELLOW);
  bothReverse();
  delay(3000);
  stopAll();
  
  digitalWrite(LED, LOW);
  displayRestState();
  delay(1000);
}

void loop() {
  // Complete test sequence
  testMotor1();
  delay(1000);
  
  testMotor2();
  delay(1000);
  
  testBothMotors();
  delay(1000);
  
  // Show completion
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
  // Loop repeats
}
