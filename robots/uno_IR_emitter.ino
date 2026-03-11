/*
 * IR Emitter — Proximity Test for Arduino Uno
 *
 * Drives a 940nm IR LED in configurable modes for distance testing.
 * Works standalone on battery (defaults to steady ON).
 * If USB serial is connected, single-key commands switch modes.
 *
 * Serial Monitor: 115200 baud, "No line ending"
 *
 * Wiring:
 *   D2 ─── 200 Ω resistor ─── IR LED anode (long leg)
 *                               IR LED cathode (short leg) ─── GND
 */

const int IR_PIN = 2;

enum Mode { STEADY_ON, TOGGLE_50HZ, STEADY_OFF };
Mode mode = STEADY_ON;

void applyMode() {
  switch (mode) {
    case STEADY_ON:
      digitalWrite(IR_PIN, HIGH);
      break;
    case STEADY_OFF:
      digitalWrite(IR_PIN, LOW);
      break;
    case TOGGLE_50HZ:
      break;
  }
}

void setup() {
  pinMode(IR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  applyMode();

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }

  Serial.println();
  Serial.println("=========================");
  Serial.println("  IR Emitter — Test Mode");
  Serial.println("=========================");
  printHelp();
  Serial.println(">> Mode: Steady ON (default)");
  Serial.println();
}

void printHelp() {
  Serial.println("Keys:");
  Serial.println("  1 = Steady ON");
  Serial.println("  2 = 50 Hz toggle (for ambient cancellation)");
  Serial.println("  3 = OFF");
  Serial.println("  h = Help");
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        mode = STEADY_ON;
        applyMode();
        Serial.println(">> Mode: Steady ON");
        break;
      case '2':
        mode = TOGGLE_50HZ;
        Serial.println(">> Mode: 50 Hz toggle");
        break;
      case '3':
        mode = STEADY_OFF;
        applyMode();
        Serial.println(">> Mode: OFF");
        break;
      case 'h': case 'H':
        printHelp();
        break;
      case '\n': case '\r': case ' ':
        break;
      default:
        Serial.print(">> Unknown: '");
        Serial.print(cmd);
        Serial.println("'");
        break;
    }
  }

  if (mode == TOGGLE_50HZ) {
    digitalWrite(IR_PIN, HIGH);
    delayMicroseconds(10000);
    digitalWrite(IR_PIN, LOW);
    delayMicroseconds(10000);
  }

  // Heartbeat blink
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
