/*
 * Nano IR Receiver — TSOP38238 / TSOP4838 test
 * Arduino Nano 33 BLE (USB power, 3.3 V logic)
 *
 * Reads a salvaged 3-pin 38 kHz IR demodulator and prints detect/idle on Serial.
 * Pair with Uno_IR_emitter on a separate board — no shared GND wire needed.
 *
 * Serial Monitor: 115200 baud, line ending "No line ending"
 *
 * ── HARDWARE ──────────────────────────────────────────────────────────
 *
 * TSOP module pinout — lens toward you, legs pointing down.
 * Many modules (left → right): OUT · GND · VCC
 * Salvaged clone parts may NOT match donor PCB silkscreen — wire the module
 * so the *original remote* gives solid DETECT, then leave it.
 *
 * Required connections:
 *
 *   TSOP pin 1 (OUT) ─── Nano D3
 *   TSOP pin 2 (GND) ─── Nano GND
 *   TSOP pin 3 (VCC) ─── Nano 3.3V
 *
 * Recommended (reduces intermittent glitches — present on donor YJ-3066R-X PCB):
 *
 *   3.3V ─── 100 nF ceramic ─── GND        (at the module, close to VCC pin)
 *   3.3V ─── 4.7 kΩ ─── D3 ─── TSOP OUT    (external pull-up; optional if
 *                                             INPUT_PULLUP alone is stable)
 *
 * Do NOT connect TSOP VCC to 5 V — Nano GPIO/logic is 3.3 V.
 *
 * ASCII:
 *
 *   Nano 3.3V ─── [100nF] ─── GND
 *        │
 *        ├─── [4.7k optional] ─── D3 ─── TSOP OUT (pin 1)
 *   Nano GND  ─────────────────────── TSOP GND  (pin 2)
 *   Nano 3.3V ─────────────────────── TSOP VCC  (pin 3)
 *
 * Behaviour:
 *   idle   — no 38 kHz IR (output idles HIGH; open-collector + pull-up)
 *   DETECT — 38 kHz burst received (output pulls LOW)
 *
 * ── POLLING RATE ────────────────────────────────────────────────────────
 *
 * Uno_IR_emitter bursts 9 ms ON / 91 ms OFF (100 ms cycle). delay(100)
 * in this loop aliases with that cycle → constant idle. POLL_MS ≤ 20 ms
 * tested working. For production: CHANGE interrupt or "any LOW in 100 ms".
 */

const int TSOP_PIN = 3;

const int POLL_MS = 20;   // must stay ≤ ~20 ms with Uno_IR_emitter bursts

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  pinMode(TSOP_PIN, INPUT_PULLUP);
  Serial.println("TSOP test — LOW = 38 kHz IR detected");
  Serial.println("(pair with Uno_IR_emitter; poll <= 20 ms)");
}

void loop() {
  Serial.println(digitalRead(TSOP_PIN) == LOW ? "DETECT" : "idle");
  delay(POLL_MS);
}
