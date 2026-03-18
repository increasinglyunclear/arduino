/*
 * XIAO ESP32-S3 PLUS — Motor Test
 * Kevin Walker 18 MAR 2026
 *
 * DFRobot DFR0399 75:1 servo-style motors.
 * PPM signal: 500µs=full CW, 1500µs=stop, 2500µs=full CCW.
 *
 * Wiring:
 *   Motor 1 signal → D4 (IO5)
 *   Motor 2 signal → D6 (IO43)
 *   Motor VCC      → VUSB (5V from USB) or LiPo+ (3.7V, min 3.5V)
 *   Motor GND      → GND
 *
 * Runs automatic test sequence, then accepts serial commands:
 *   1 = motor 1 forward    2 = motor 1 reverse
 *   3 = motor 2 forward    4 = motor 2 reverse
 *   5 = both forward       6 = both reverse
 *   0 = stop all
 */

#include <ESP32Servo.h>

#define MOTOR1_PIN D4   // IO5
#define MOTOR2_PIN D6   // IO43

const int STOP = 1500;
const int FWD  = 500;   // full speed clockwise
const int REV  = 2500;  // full speed counter-clockwise
const int HALF_FWD = 1000;
const int HALF_REV = 2000;

Servo motor1;
Servo motor2;

void stopAll() {
  motor1.writeMicroseconds(STOP);
  motor2.writeMicroseconds(STOP);
}

void runTest(const char* label, int m1, int m2, unsigned long ms) {
  Serial.printf("  %s ...", label);
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  delay(ms);
  stopAll();
  delay(500);
  Serial.println(" done");
}

void setup() {
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);  // LED on (active LOW)

  Serial.begin(115200);
  delay(1000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor1.attach(MOTOR1_PIN, 500, 2500);
  motor2.attach(MOTOR2_PIN, 500, 2500);
  stopAll();

  Serial.println("Motor Test");
  Serial.println("==========");
  Serial.println("Running automatic test (half speed, 1 second each)...\n");

  runTest("Motor 1 forward",  HALF_FWD, STOP, 1000);
  runTest("Motor 1 reverse",  HALF_REV, STOP, 1000);
  runTest("Motor 2 forward",  STOP, HALF_FWD, 1000);
  runTest("Motor 2 reverse",  STOP, HALF_REV, 1000);
  runTest("Both forward",     HALF_FWD, HALF_FWD, 1000);
  runTest("Both reverse",     HALF_REV, HALF_REV, 1000);

  Serial.println("\nAutomatic test done.\n");
  Serial.println("Serial commands:");
  Serial.println("  1=M1 fwd  2=M1 rev  3=M2 fwd  4=M2 rev");
  Serial.println("  5=both fwd  6=both rev  0=stop");
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  switch (c) {
    case '1':
      Serial.println("Motor 1 forward");
      motor1.writeMicroseconds(HALF_FWD);
      break;
    case '2':
      Serial.println("Motor 1 reverse");
      motor1.writeMicroseconds(HALF_REV);
      break;
    case '3':
      Serial.println("Motor 2 forward");
      motor2.writeMicroseconds(HALF_FWD);
      break;
    case '4':
      Serial.println("Motor 2 reverse");
      motor2.writeMicroseconds(HALF_REV);
      break;
    case '5':
      Serial.println("Both forward");
      motor1.writeMicroseconds(HALF_FWD);
      motor2.writeMicroseconds(HALF_FWD);
      break;
    case '6':
      Serial.println("Both reverse");
      motor1.writeMicroseconds(HALF_REV);
      motor2.writeMicroseconds(HALF_REV);
      break;
    case '0':
      Serial.println("Stop");
      stopAll();
      break;
  }
}
