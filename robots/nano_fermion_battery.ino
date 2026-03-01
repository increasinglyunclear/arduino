// Fermion HR8833 Test - standalone with 3.7V LiPo battery
// Kevin Waker 01 Mar 2026

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

const int M1A = 9;
const int M1B = 7;
const int M2A = 8;
const int M2B = 6;

void setup() {
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  
  // Initialize LED matrix
  matrix.begin(0x70);
  matrix.setBrightness(4);
  
  // Show "OK" on startup
  matrix.clear();
  matrix.setCursor(0, 0);
  matrix.setTextColor(LED_GREEN);
  matrix.print("OK");
  matrix.writeDisplay();
  delay(2000);
}

void stopAll() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
}

void showStatus(const char* text, uint8_t color) {
  matrix.clear();
  matrix.setCursor(0, 0);
  matrix.setTextColor(color);
  matrix.print(text);
  matrix.writeDisplay();
}

void loop() {
  // Motor 1 Forward
  showStatus("M1", LED_GREEN);
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  delay(2000);
  stopAll();
  delay(500);
  
  // Motor 2 Forward
  showStatus("M2", LED_GREEN);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  delay(2000);
  stopAll();
  delay(500);
  
  // Both Forward
  showStatus(">>", LED_GREEN);
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  delay(3000);
  stopAll();
  delay(500);
  
  // Both Reverse
  showStatus("<<", LED_YELLOW);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
  delay(3000);
  stopAll();
  
  // Complete
  showStatus("OK", LED_GREEN);
  delay(2000);
}
