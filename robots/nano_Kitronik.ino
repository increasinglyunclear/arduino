/*
 * Arduino Nano 33 BLE Sense Rev 1
 * Motor control via Kitronik 5108 (DRV8833) motor driver board.
 * Kevin Walker 20 Feb 2026
 *
 * Hardware Connections:
 *
 *   Nano 33 BLE          Kitronik 5108
 *   -----------          -------------
 *   D2  ──────────────── AIN1   (Motor A control 1)
 *   D3  ──────────────── AIN2   (Motor A control 2)
 *   D4  ──────────────── BIN1   (Motor B control 1)
 *   D5  ──────────────── BIN2   (Motor B control 2)
 *   D6  ──────────────── nSLEEP (driver enable)
 *   GND ──────────────── GND
 *                        Vcc ── 3.7V LiPo battery positive
 *                        GND ── Battery negative (common ground with Arduino)
 *
 * Serial Commands (115200 baud):
 *   af / ar / as / ab   Motor A: forward/reverse/stop/brake
 *   bf / br / bs / bb   Motor B: forward/reverse/stop/brake
 *   ff / rr / ss        Both: forward/reverse/stop
 *   demo                Run motor demo sequence
 *   help                Show commands
 *
 * NOTE: PWM speed control disabled to test USB serial stability.
 *       Motors run at full speed only (digitalWrite, no analogWrite).
 */

// Motor A pins
#define AIN1 2
#define AIN2 3

// Motor B pins
#define BIN1 4
#define BIN2 5

// DRV8833 nSLEEP pin
#define NSLEEP_PIN 6

// Built-in LED (active LOW)
#define LED_PIN LED_BUILTIN

void setup() {
  // CRITICAL: Motor pins LOW immediately to prevent spurious spinning
  pinMode(AIN1, OUTPUT);  digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT);  digitalWrite(AIN2, LOW);
  pinMode(BIN1, OUTPUT);  digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT);  digitalWrite(BIN2, LOW);

  // Keep motor driver asleep until ready
  pinMode(NSLEEP_PIN, OUTPUT);
  digitalWrite(NSLEEP_PIN, LOW);

  // Built-in LED for heartbeat (using digitalWrite only, no analogWrite)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // OFF (active LOW)

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("=================================");
  Serial.println("Robot Controller");
  Serial.println("Nano 33 BLE Sense + Kitronik 5108");
  Serial.println("=================================");
  Serial.println();
  Serial.println("[Motor] Pins: A(D2,D3) B(D4,D5) nSLEEP(D6)");

  // Wake motor driver
  digitalWrite(NSLEEP_PIN, HIGH);
  Serial.println("[Motor] Driver enabled");

  Serial.println();
  Serial.println("Type 'help' for commands.");
  Serial.println("=================================");
  Serial.println();
}

// Serial command buffer
char cmdBuf[32];
int cmdLen = 0;

// Heartbeat
unsigned long lastHeartbeat = 0;
unsigned long cmdCount = 0;

void loop() {
  // Heartbeat every 3 seconds (digitalWrite only, no analogWrite)
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    digitalWrite(LED_PIN, LOW);  // ON
    delay(50);
    digitalWrite(LED_PIN, HIGH); // OFF
    Serial.print("[hb] cmds=");
    Serial.print(cmdCount);
    Serial.print(" avail=");
    Serial.println(Serial.available());
  }

  // Read serial input
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        for (int i = 0; i < cmdLen; i++) {
          if (cmdBuf[i] >= 'A' && cmdBuf[i] <= 'Z') cmdBuf[i] += 32;
        }
        cmdCount++;
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ── Motor Control (digitalWrite only, no analogWrite) ──────────

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

// ── Command Handler ────────────────────────────────────────────

void handleCommand(const char* cmd) {
  if (strcmp(cmd, "help") == 0) {
    printHelp();
  } else if (strcmp(cmd, "af") == 0) {
    motorForward('A');
    Serial.println("[A] Forward");
  } else if (strcmp(cmd, "ar") == 0) {
    motorReverse('A');
    Serial.println("[A] Reverse");
  } else if (strcmp(cmd, "as") == 0) {
    motorStop('A');
    Serial.println("[A] Stop");
  } else if (strcmp(cmd, "ab") == 0) {
    motorBrake('A');
    Serial.println("[A] Brake");
  } else if (strcmp(cmd, "bf") == 0) {
    motorForward('B');
    Serial.println("[B] Forward");
  } else if (strcmp(cmd, "br") == 0) {
    motorReverse('B');
    Serial.println("[B] Reverse");
  } else if (strcmp(cmd, "bs") == 0) {
    motorStop('B');
    Serial.println("[B] Stop");
  } else if (strcmp(cmd, "bb") == 0) {
    motorBrake('B');
    Serial.println("[B] Brake");
  } else if (strcmp(cmd, "ff") == 0) {
    motorForward('A');
    motorForward('B');
    Serial.println("[AB] Forward");
  } else if (strcmp(cmd, "rr") == 0) {
    motorReverse('A');
    motorReverse('B');
    Serial.println("[AB] Reverse");
  } else if (strcmp(cmd, "ss") == 0) {
    motorStop('A');
    motorStop('B');
    Serial.println("[AB] Stop");
  } else if (strcmp(cmd, "demo") == 0) {
    runMotorDemo();
  } else {
    Serial.print("? ");
    Serial.println(cmd);
  }
}

void printHelp() {
  Serial.println("af/ar/as/ab  Motor A: fwd/rev/stop/brake");
  Serial.println("bf/br/bs/bb  Motor B: fwd/rev/stop/brake");
  Serial.println("ff/rr/ss     Both: fwd/rev/stop");
  Serial.println("demo         Motor demo");
}

void runMotorDemo() {
  Serial.println("-- Demo --");

  Serial.println("A fwd");
  motorForward('A');
  delay(1000);
  motorStop('A');
  delay(300);

  Serial.println("A rev");
  motorReverse('A');
  delay(1000);
  motorStop('A');
  delay(300);

  Serial.println("B fwd");
  motorForward('B');
  delay(1000);
  motorStop('B');
  delay(300);

  Serial.println("AB fwd");
  motorForward('A');
  motorForward('B');
  delay(1000);

  Serial.println("AB rev");
  motorReverse('A');
  motorReverse('B');
  delay(1000);

  Serial.println("Brake");
  motorBrake('A');
  motorBrake('B');
  delay(500);

  motorStop('A');
  motorStop('B');
  Serial.println("-- Done --");
}
