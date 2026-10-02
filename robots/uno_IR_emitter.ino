/*
 * Uno IR Emitter — 38 kHz burst beacon for TSOP38238 / TSOP4838 receivers
 * Arduino Uno (USB power)
 *
 * Drives a salvaged 940 nm IR LED through a BC547 NPN transistor.
 * Matches TV-remote style signalling: high peak current in short bursts,
 * not continuous DC or slow toggle.
 *
 * Serial Monitor: not required (no serial output).
 *
 * ── HARDWARE ──────────────────────────────────────────────────────────
 *
 * BC547 pinout — flat face toward you, legs pointing down:
 *
 *      ┌─────────┐
 *      │ BC547   │
 *      └─────────┘
 *       C   B   E        C = Collector (left)
 *                          B = Base      (middle)
 *                          E = Emitter   (right)
 *
 * IR LED — clear lens = 940 nm emitter:
 *   anode  (+) long leg
 *   cathode (−) short leg
 *
 * Connections (4 signal paths):
 *
 *   1. Uno D2 ─── 1 kΩ ─── BC547 Base (middle)
 *
 *   2. Uno 5V ─── 10 Ω ─── IR LED anode (+)     ← current-limit; see below
 *   3. IR LED cathode (−) ─── BC547 Collector (left)
 *   4. BC547 Emitter (right) ─── Uno GND
 *
 * ASCII:
 *
 *   Uno 5V ─── 10Ω ─── IR LED (+) ─── IR LED (−) ─── BC547 C
 *                                         BC547 B ←── 1kΩ ←── Uno D2
 *                                         BC547 E ──────────── Uno GND
 *
 * ── RESISTOR NOTES ────────────────────────────────────────────────────
 *
 * LED current-limit (5V → anode):
 *   10 Ω  ≈ 300 mA peak during burst — tested; matches salvaged remote
 *         (original remote PCB uses 5.6 Ω + pulsed drive, not continuous).
 *   22 Ω  ≈ 160 mA peak — gentler if 10 Ω feels hot.
 *   100 Ω ≈  36 mA peak — too dim for room-range TSOP detection.
 *
 *   NEVER run 10 Ω with continuous tone() — only with the 9 ms bursts below.
 *
 * Base resistor (D2 → Base): 1 kΩ (700 Ω–2 kΩ all OK).
 *
 * ── SMOKE TEST (before uploading this sketch) ───────────────────────────
 *
 *   void setup() { pinMode(2, OUTPUT); digitalWrite(2, HIGH); }
 *   void loop() {}
 *
 *   Check IR LED with front phone camera — faint purple glow = wiring OK.
 *   DC HIGH will NOT trigger a TSOP receiver; it needs 38 kHz modulation.
 */

const int IR_PIN = 2;

const int CARRIER_HZ = 38000;   // TSOP38xx passband ~36–40 kHz

const int BURST_ON_MS  = 9;     // ON window — like a remote packet
const int BURST_OFF_MS = 91;    // OFF gap — receiver AGC recovery

void setup() {
  pinMode(IR_PIN, OUTPUT);
}

void loop() {
  tone(IR_PIN, CARRIER_HZ);
  delay(BURST_ON_MS);
  noTone(IR_PIN);
  delay(BURST_OFF_MS);
}
