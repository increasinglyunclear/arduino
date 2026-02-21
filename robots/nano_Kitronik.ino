/*
 * Arduino Nano 33 BLE Sense Rev 1 + Kitronik 5108 (DRV8833)
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
 *   Power the Nano via LiPo positive to the 5V pin, negative to GND.
 */

#define AIN1 2
#define AIN2 3
#define BIN1 4
#define BIN2 5
#define NSLEEP_PIN 6
#define LED_PIN LED_BUILTIN

void setup() {
  // Motor pins LOW immediately to prevent spurious spinning during boot
  pinMode(AIN1, OUTPUT);  digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT);  digitalWrite(AIN2, LOW);
  pinMode(BIN1, OUTPUT);  digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT);  digitalWrite(BIN2, LOW);

  // Keep motor driver asleep until ready
  pinMode(NSLEEP_PIN, OUTPUT);
  digitalWrite(NSLEEP_PIN, LOW);

  // LED to signal startup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // OFF (active LOW)

  // Blink LED to confirm board is running
  blinkLED(3);

  // Startup delay so you can set the robot down
  delay(2000);

  // Wake motor driver
  digitalWrite(NSLEEP_PIN, HIGH);
}

void loop() {
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
