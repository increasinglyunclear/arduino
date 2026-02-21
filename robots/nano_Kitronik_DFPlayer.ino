/*
 * Arduino Nano 33 BLE Sense Rev 1 + Kitronik 5108 + DFPlayer Mini
 * Runs a repeating motor demo with audio playback, powered by 3.7V LiPo.
 * Kevin Walker 21 Feb 2026
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
 *   Power the Nano via LiPo positive to the 5V pin, negative to GND.
 *
 * SD Card: FAT32 formatted, files named 0001.mp3, 0002.mp3, 0003.mp3 in root.
 *
 * Required Library:
 *   DFRobotDFPlayerMini (install via Arduino Library Manager)
 */

#include <DFRobotDFPlayerMini.h>

// Motor A pins
#define AIN1 2
#define AIN2 3

// Motor B pins
#define BIN1 4
#define BIN2 5

// DRV8833 nSLEEP pin
#define NSLEEP_PIN 6

#define LED_PIN LED_BUILTIN

// DFPlayer communicates over Serial1 (D0=RX, D1=TX on Nano 33 BLE)
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;

void setup() {
  // Motor pins LOW immediately to prevent spurious spinning during boot
  pinMode(AIN1, OUTPUT);  digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT);  digitalWrite(AIN2, LOW);
  pinMode(BIN1, OUTPUT);  digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT);  digitalWrite(BIN2, LOW);

  // Keep motor driver asleep until ready
  pinMode(NSLEEP_PIN, OUTPUT);
  digitalWrite(NSLEEP_PIN, LOW);

  // LED for status
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // OFF (active LOW)

  // Init DFPlayer on hardware Serial1
  Serial1.begin(9600);
  delay(500);

  if (dfPlayer.begin(Serial1)) {
    dfPlayerReady = true;
    dfPlayer.volume(20); // 0-30
  }

  // Blink LED: 3 times = all good, rapid blink = DFPlayer failed
  if (dfPlayerReady) {
    blinkLED(3);
  } else {
    for (int i = 0; i < 10; i++) blinkLED(1);
  }

  // Startup delay so you can set the robot down
  delay(2000);

  // Wake motor driver
  digitalWrite(NSLEEP_PIN, HIGH);
}

// Track index for cycling through the 3 audio files
int currentTrack = 1;

void playNextTrack() {
  if (!dfPlayerReady) return;
  dfPlayer.play(currentTrack);
  currentTrack++;
  if (currentTrack > 3) currentTrack = 1;
}

void loop() {
  // Play a track at the start of each cycle
  playNextTrack();
  delay(500);

  // Motor A forward
  motorForward('A');
  delay(1000);
  motorStop('A');
  delay(500);

  // Motor A reverse
  motorReverse('A');
  delay(1000);
  motorStop('A');
  delay(500);

  // Play next track
  playNextTrack();
  delay(500);

  // Motor B forward
  motorForward('B');
  delay(1000);
  motorStop('B');
  delay(500);

  // Motor B reverse
  motorReverse('B');
  delay(1000);
  motorStop('B');
  delay(500);

  // Play next track
  playNextTrack();
  delay(500);

  // Both forward
  motorForward('A');
  motorForward('B');
  delay(1000);
  motorStop('A');
  motorStop('B');
  delay(500);

  // Both reverse
  motorReverse('A');
  motorReverse('B');
  delay(1000);

  // Brake
  motorBrake('A');
  motorBrake('B');
  delay(500);

  // Stop and pause before next cycle
  motorStop('A');
  motorStop('B');

  // Blink LED to mark end of cycle
  blinkLED(1);

  delay(2000);
}

// ── Motor Control ──────────────────────────────────────────────

void motorForward(char motor) {
  if (motor == 'A') {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
}

void motorReverse(char motor) {
  if (motor == 'A') {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
}

void motorStop(char motor) {
  if (motor == 'A') {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

void motorBrake(char motor) {
  if (motor == 'A') {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
  }
}

// ── LED ────────────────────────────────────────────────────────

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, LOW);  // ON
    delay(150);
    digitalWrite(LED_PIN, HIGH); // OFF
    delay(150);
  }
}
