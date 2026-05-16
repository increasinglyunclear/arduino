/*
 * Robot-A (Emitter) — v4 - THE ONE WITH THE CUBE.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <DFRobotDFPlayerMini.h>
#include "LedControl.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// DEBUG BUILD: serial logging enabled so Arduino Serial Monitor shows boot
// AT responses, UWB ranging state, and per-command logs. To silence for
// untethered runs (saves a few µA and a little flash), change to:
//   #define SERIAL_LOG if (0) Serial
// which compiles every SERIAL_LOG.* call down to a no-op.
#define SERIAL_LOG Serial
HardwareSerial SERIAL_AT(2);

#define RESET_PIN   16
#define IO_RXD2     18
#define IO_TXD2     17
#define I2C_SDA     39
#define I2C_SCL     38
#define DFPLAYER_TX 1
#define DFPLAYER_RX 2

// ── Peripheral enable flags ──────────────────────────────
// See matching block in mauwb_receiver.ino for rationale. Set to 0 to
// skip init and all runtime calls — useful on a brownout-prone supply
// while debugging pure motion / seek.
#define ENABLE_DFPLAYER 0

// LEAN_MODE:
//   Suspend UWB ranging, BNO055 IMU, and peer-scan trilateration. Only
//   the BLE command channel (AnchorCmd) and motor / matrix output stay
//   live. See the matching block in mauwb_receiver.ino for rationale.
//   On T0 this has extra bite: T0 was configured as a UWB anchor
//   (AT+SETCFG=1,1,1,1), whose receiver is always-on and the dominant
//   steady-state current draw that's been driving the brownouts. Holding
//   the DW3000 STM32 in reset kills that load outright.
#define LEAN_MODE 1

// ENABLE_BUMP_DETECT:
//   Keep the BNO055 IMU alive even under LEAN_MODE so accelerometer-based
//   impact detection can fire during browser-driven sequence playback.
//   BNO055 in IMU mode draws ~12 mA — small compared to the ~100 mA UWB
//   load that LEAN_MODE eliminates. Set to 0 to drop another ~12 mA if
//   the supply still browns out and you're OK running blind into walls.
#define ENABLE_BUMP_DETECT 0

// ── Autoplay: hard-coded sequence on boot ───────────────
// With anchor/browser unavailable, the robot can run a preloaded sequence
// on its own. Set AUTOPLAY to 1, pick a sequence number, flash both
// robots with the same or different sequences. See AUTOPLAY_TABLE below
// for the step data (ported verbatim from position_map/index.html's
// BUILTIN_SEQUENCES). Each robot waits AUTOPLAY_DELAY_MS after boot
// before moving, so you can place them after power-on.
//   1 = Finding          5 = I See You
//   2 = Missed Connections  6 = Circling
//   3 = Levy A           7 = Chemotaxis
//   4 = Levy B
#define AUTOPLAY             1
#define AUTOPLAY_DELAY_MS    10000
#define AUTOPLAY_SEQUENCE    9
#define AUTOPLAY_REPEAT      1   // 1 = once, 2+ = repeat N times, 0 = loop forever

#define BNO_SDA     5
#define BNO_SCL     6
#define MOTOR1_PIN  7
#define MOTOR2_PIN  8
#define IR_SENSE_PIN 42
#define IR_DETECTED  HIGH
#define MATRIX_DIN  9
#define MATRIX_CLK  10
#define MATRIX_CS   11

// ── Motor tuning ─────────────────────────────────────────
const int MOTOR_STOP = 1500;
const int M1_FWD  = 2000;
const int M2_FWD  = 1000;
const int BIAS = 0;

const float TRIM_GAIN    = 6.0;
const int   TRIM_MAX     = 300;
const float DEADBAND_DEG = 1.5;

int motor1_us = 0, motor2_us = 0;
portMUX_TYPE pwmMux = portMUX_INITIALIZER_UNLOCKED;

// Motor bias / trim (set at runtime via command 0x11).
// Range [-50, +50] percent. Applied in updateServos() so *every* motor
// command path — manual drive, walk, turn, seek, wander, bump recovery,
// sequence playback — gets the same correction. The scalar weakens one
// wheel relative to the other by scaling its (us - MOTOR_STOP) delta:
//   bias > 0  → cut M2 by bias%    (robot was veering right → boost effective M1)
//   bias < 0  → cut M1 by |bias|%  (robot was veering left  → boost effective M2)
// We only *cut* (never boost above 100%) to avoid pushing a wheel beyond
// the calibrated FWD/REV endpoints.
int8_t motorBiasPct = -4;

// ── Distance-gradient steering ───────────────────────────
float prevCheckDist = 0;
unsigned long prevCheckTime = 0;
const unsigned long DIST_CHECK_MS  = 2000;
const float FARTHER_THRESH = 5.0;
int   nudgeDir   = 1;
int   nudgeCount = 0;
const float NUDGE_DEG = 3.0;
const int   MAX_NUDGE = 4;

// ── Hardware ─────────────────────────────────────────────
TwoWire bnoWire(1);
Adafruit_BNO055* bno = nullptr;
bool imuOK = false;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;
LedControl* lc = nullptr;

const uint8_t arrows[8][8] = {
  {0x10, 0x28, 0x44, 0x10, 0x10, 0x10, 0x10, 0x00},   // 0: up
  {0x02, 0x02, 0x06, 0x08, 0x10, 0x20, 0x40, 0x00},   // 1: up-right
  {0x00, 0x04, 0x02, 0xFF, 0x02, 0x04, 0x00, 0x00},   // 2: right
  {0x00, 0x40, 0x20, 0x10, 0x08, 0x06, 0x02, 0x02},   // 3: down-right
  {0x00, 0x10, 0x10, 0x10, 0x10, 0x44, 0x28, 0x10},   // 4: down
  {0x00, 0x02, 0x04, 0x08, 0x10, 0x60, 0x40, 0x40},   // 5: down-left
  {0x00, 0x20, 0x40, 0xFF, 0x40, 0x20, 0x00, 0x00},   // 6: left
  {0x40, 0x40, 0x60, 0x10, 0x08, 0x04, 0x02, 0x00},   // 7: up-left
};

// ── State ────────────────────────────────────────────────
enum Phase { PH_WAIT_UWB, PH_DRIVE, PH_ARRIVED, PH_MANUAL, PH_SEEK, PH_WANDER, PH_PATH };
Phase phase = PH_WAIT_UWB;
float targetHdg = 0;
const float ARRIVE_DIST = 10.0;
unsigned long driveStartTime = 0;
const unsigned long MIN_DRIVE_MS = 3000;

// ── Path following ──────────────────────────────────────
// Waypoints drawn by the user on the map (int16 cm in anchor-frame coords).
// Robot navigates waypoint-by-waypoint; advances when within WP_ARRIVE_DIST.
#define MAX_PATH_WP 2
struct Waypoint { int16_t x, y; };
Waypoint pathWp[MAX_PATH_WP];
uint8_t pathLen = 0, pathIdx = 0;
const float WP_ARRIVE_DIST = 25.0f;

// ── Per-anchor Kalman filters + trilateration ───────────
const float KF_Q = 9.0;
const float KF_R = 16.0;

struct AnchorKF {
  float x = 0, P = 1000.0;
  bool init = false;
  void feed(float z) {
    if (!init) { x = z; P = KF_R; init = true; return; }
    P += KF_Q;
    float K = P / (P + KF_R);
    x += K * (z - x);
    P *= (1.0 - K);
  }
  void reset() { init = false; P = 1000.0; }
};

AnchorKF kfAnc[2];
unsigned long lastUwbTime = 0;
const unsigned long UWB_TIMEOUT_MS = 3000;

#define ANCHOR_DIST_CM 200.0f  // A0↔A1 distance — CHANGE TO MATCH YOUR SETUP

float tagX = 0, tagY = 0;
bool posValid = false;

// Peer robot position (received via BLE scanning)
volatile float otherX = 0, otherY = 0;
volatile bool otherPosValid = false;
volatile unsigned long lastOtherUpdate = 0;
const unsigned long OTHER_TIMEOUT_MS = 10000;

float mapBasisHdg = 0;

#define MY_ROBOT_ID 0
volatile uint8_t lastAnchorSeq = 0;
uint8_t driveSpeedPct = 70;

// Wander state
float wanderHdg = 0;
unsigned long wanderNextTurn = 0;

// Seek arrival + overshoot
const float SEEK_ARRIVE_DIST = 25.0f;  // UWB accuracy floor ~10–30 cm
const int   OVERSHOOT_CM = 10;
unsigned long seekOvershootUntil = 0;

// Auto-calibration (observe motion vs compass to refine mapBasisHdg)
float calPrevX = 0, calPrevY = 0;
unsigned long calPrevTime = 0;
bool calTracking = false;

// Seek distance-gradient correction
float seekPrevDist = 0;
unsigned long seekPrevDistTime = 0;

// ── Seek motion-probe auto-calibration ──────────────────
// On SEEK entry we drive straight forward for a short distance and measure the
// ACTUAL map-frame direction of motion via UWB. Then mapBasisHdg = C0 − α_obs
// gives an empirically-correct heading basis without the user having to align
// the robot by eye. This is much more accurate than manual calibration.
bool seekProbeArmed = false;
bool seekProbing = false;
unsigned long seekProbeStartT = 0;
float seekProbeX0 = 0, seekProbeY0 = 0;
float seekProbeC0 = 0;
const float PROBE_MIN_MOVE_CM    = 15.0f;   // minimum motion to trust the probe
const float PROBE_TARGET_MOVE_CM = 25.0f;   // end probe once we've moved this far
const unsigned long PROBE_TIMEOUT_MS = 4000;

// ── Bump / stall recovery ───────────────────────────────
// Triggered by physical obstacle (robot commanded forward but not moving)
// OR by peer proximity (wander only). Behavior: back up ~800 ms, then spin ~270°.
enum BumpState { BUMP_IDLE, BUMP_BACKING, BUMP_TURNING };
BumpState bumpState = BUMP_IDLE;
unsigned long bumpEndTime = 0;
float bumpPrevHdg = 0, bumpCumTurn = 0;
int bumpDir = 1;                    // +1 = CCW/left, -1 = CW/right
Phase bumpReturnPhase = PH_MANUAL;

float stallPrevX = 0, stallPrevY = 0;
unsigned long stallPrevTime = 0;
// Stall detection is the *only* bump detector now (accelerometer-based impact
// detection disabled — too many false triggers from motor vibration at the
// new soft-start PWM levels). Threshold just above UWB jitter floor.
const float STALL_MIN_MOVE_CM = 5.0;         // ~half of UWB noise — triggers fast on real stall
const unsigned long STALL_WINDOW_MS = 900;   // quicker reaction

// ── Accelerometer-based impact detection ────────────────
// BNO055 linear-accel (gravity removed) spikes on wall impact. Combined with
// the position-based stall check above, this gives both fast (accel) and
// reliable (position) collision detection.
unsigned long drivingSince = 0;
unsigned long lastAccelCheck = 0;
const float ACCEL_IMPACT_MPS2 = 1.5f;       // more sensitive (was 2.0)
const unsigned long DRIVE_BLANK_MS = 400;   // shorter blanking (was 500)
const unsigned long ACCEL_CHECK_MS = 25;    // ~40 Hz (was ~33)

// Direction-aware check: is the robot actually being commanded to drive forward?
// The motor pulse for "forward" is >MOTOR_STOP for one wheel and <MOTOR_STOP for
// the other (they're mirrored), so a simple `> MOTOR_STOP+50` on both wheels
// would NEVER be true — the old check was silently broken.
inline bool isDrivingForward() {
  int m1Delta = (M1_FWD > MOTOR_STOP) ? (motor1_us - MOTOR_STOP) : (MOTOR_STOP - motor1_us);
  int m2Delta = (M2_FWD > MOTOR_STOP) ? (motor2_us - MOTOR_STOP) : (MOTOR_STOP - motor2_us);
  return motor1_us > 0 && motor2_us > 0 && m1Delta > 50 && m2Delta > 50;
}

// ── No-go obstacle (line segment drawn on map) ──────────
// User draws a line on the map representing an obstacle (wall, drop, etc).
// Robot triggers bump recovery if its position is within NOGO_THRESH_CM of the segment.
// All zero → disabled.
volatile int16_t noGoX1 = 0, noGoY1 = 0, noGoX2 = 0, noGoY2 = 0;
unsigned long lastNoGoTrig = 0;
const unsigned long NOGO_COOLDOWN_MS = 4000;
const float NOGO_THRESH_CM = 25.0f;

// ── Seek wrong-way counter ──────────────────────────────
uint8_t seekWrongCount = 0;

float& kf_x = kfAnc[0].x;
bool kfInit = false;

void trilaterateTag() {
  if (!kfAnc[0].init || !kfAnc[1].init) { posValid = false; return; }
  float d0 = kfAnc[0].x, d1 = kfAnc[1].x;
  float D = ANCHOR_DIST_CM;
  float x = (d0 * d0 - d1 * d1 + D * D) / (2.0f * D);
  float y2 = d0 * d0 - x * x;
  if (y2 >= 0) { tagX = x; tagY = sqrtf(y2); posValid = true; }
  kfInit = true;
}

String response = "";

// ── BLE command buffer (shared by GATT + scan callbacks) ─
volatile bool cmdPending = false;
volatile uint8_t cmdBuf[12];
volatile uint8_t cmdLen = 0;

// ── BLE Peer Scan (receive other robot's position) ──────
BLEScan* pBleScan = nullptr;

class PeerScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveName()) return;
    String name = dev.getName().c_str();

#if !LEAN_MODE
    // Peer trilateration path: parses T1's broadcast UWB distances to
    // compute where T1 is. Suspended in LEAN_MODE — T1 isn't ranging at
    // all, so its pd0/pd1 bytes would be zero anyway.
    if (name == "RobotB-Rx") {
      if (!dev.haveManufacturerData()) return;
      String mfRaw = dev.getManufacturerData();
      const uint8_t* mf = (const uint8_t*)mfRaw.c_str();
      int len = mfRaw.length();
      int off = (len >= 8 && mf[0] == 0xFF && mf[1] == 0xFF) ? 2 : 0;
      if (len - off < 6) return;
      uint16_t pd0 = ((uint16_t)mf[off] << 8) | mf[off + 1];
      uint16_t pd1 = ((uint16_t)mf[off + 2] << 8) | mf[off + 3];
      if (pd0 == 0 || pd1 == 0) return;
      float D = ANCHOR_DIST_CM;
      float fx = ((float)pd0 * pd0 - (float)pd1 * pd1 + D * D) / (2.0f * D);
      float fy2 = (float)pd0 * pd0 - fx * fx;
      if (fy2 >= 0) { otherX = fx; otherY = sqrtf(fy2); otherPosValid = true; lastOtherUpdate = millis(); }
      return;
    }
#endif

    if (name == "AnchorCmd") {
      if (!dev.haveManufacturerData()) return;
      String mfRaw = dev.getManufacturerData();
      const uint8_t* mf = (const uint8_t*)mfRaw.c_str();
      int len = mfRaw.length();
      int off = (len >= 5 && mf[0] == 0xFF && mf[1] == 0xFF) ? 2 : 0;
      if (len - off < 3) return;
      uint8_t seq = mf[off], target = mf[off + 1], dlen = mf[off + 2];
      if (dlen == 0 || len - off - 3 < dlen) return;
      if (target != MY_ROBOT_ID && target != 0xFF) return;
      if (seq == lastAnchorSeq) return;
      lastAnchorSeq = seq;
      cmdLen = min(dlen, (uint8_t)12);
      for (int i = 0; i < cmdLen; i++) cmdBuf[i] = mf[off + 3 + i];
      cmdPending = true;
    }
  }
};

// ── BLE (event-driven) ──────────────────────────────────
BLEAdvertising* pBleAdv = nullptr;
unsigned long lastBlePush = 0;
uint8_t lastBleDist  = 255;
uint8_t lastBlePhase = 0xFF;
const unsigned long BLE_HEARTBEAT_MS = 5000;
const int BLE_DIST_THRESHOLD = 8;

// ── BLE GATT Command Service ────────────────────────────
#define CMD_SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define CMD_CHAR_UUID     "12345678-1234-1234-1234-123456789def"

bool walkActive = false;
bool walkM1Turn = true;
unsigned long walkStepStart = 0;
const unsigned long WALK_STEP_MS = 250;

unsigned long lastLog = 0;

// ── Helpers ──────────────────────────────────────────────

String sendAT(const String& cmd, unsigned long timeout) {
  SERIAL_LOG.print(">> "); SERIAL_LOG.println(cmd);
  SERIAL_AT.println(cmd);
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (SERIAL_AT.available()) resp += (char)SERIAL_AT.read();
  }
  // Echo the STM32's reply (indented, trailing newlines stripped) so boot
  // sequence is debuggable over USB without a separate UART sniffer.
  String trimmed = resp; trimmed.trim();
  if (trimmed.length()) {
    int start = 0;
    for (int i = 0; i <= (int)trimmed.length(); ++i) {
      if (i == (int)trimmed.length() || trimmed[i] == '\n') {
        String ln = trimmed.substring(start, i); ln.trim();
        if (ln.length()) { SERIAL_LOG.print("   "); SERIAL_LOG.println(ln); }
        start = i + 1;
      }
    }
  }
  return resp;
}

void parseRange(const String& line) {
  int rs = line.indexOf("range:(");
  int as = line.indexOf("ancid:(");
  if (rs < 0 || as < 0) return;
  rs += 7;  as += 7;
  int re = line.indexOf(')', rs);
  int ae = line.indexOf(')', as);
  if (re < 0 || ae < 0) return;

  String rStr = line.substring(rs, re);
  String aStr = line.substring(as, ae);
  int ranges[8], ancids[8], nR = 0, nA = 0;

  int p = 0;
  while (p <= (int)rStr.length() && nR < 8) {
    int c = rStr.indexOf(',', p); if (c < 0) c = rStr.length();
    ranges[nR++] = rStr.substring(p, c).toInt();
    p = c + 1;
  }
  p = 0;
  while (p <= (int)aStr.length() && nA < 8) {
    int c = aStr.indexOf(',', p); if (c < 0) c = aStr.length();
    ancids[nA++] = aStr.substring(p, c).toInt();
    p = c + 1;
  }

  lastUwbTime = millis();
  for (int i = 0; i < min(nR, nA); i++) {
    if (ancids[i] >= 0 && ancids[i] <= 1 && ranges[i] > 0)
      kfAnc[ancids[i]].feed((float)ranges[i]);
  }
  trilaterateTag();
}

// ── Motor helpers ────────────────────────────────────────
// Soft-start: instead of snapping motor_us from STOP to FULL (500us step ≈ huge
// inrush current spike → brownout), we slew the *output* pulse width toward the
// requested value at MOTOR_SLEW_US per servo frame. A STOP→FULL ramp takes
// ~6 frames ≈ 120 ms, which is imperceptible but eliminates the current transient.
int motor1_us_out = 0, motor2_us_out = 0;
const int MOTOR_SLEW_US = 80;  // per 20ms servo frame

static inline int slewToward(int cur, int tgt) {
  if (cur == tgt) return tgt;
  if (cur == 0 && tgt > 0)      return MOTOR_STOP;       // OFF → ON: start at neutral, then ramp
  if (cur > 0  && tgt == 0)     return 0;                // → OFF: cut pulse immediately
  if (cur < tgt) return min(cur + MOTOR_SLEW_US, tgt);
  else           return max(cur - MOTOR_SLEW_US, tgt);
}

// Apply motorBiasPct to a single motor's target pulse.
// `us`==0 (off) and `us`==MOTOR_STOP (neutral) are preserved — no trim is
// applied to an idle or neutral wheel.
//
// Symmetric split: positive bias boosts M1 and cuts M2 by the same amount;
// negative bias does the reverse. This gives ~2x the correction authority
// of the older cut-only scheme at the same slider value, and works well
// at any partial throttle because the net differential scales with the
// commanded speed.
//   mult = 100 + bias   for M1
//   mult = 100 - bias   for M2
// At bias=+30 and 60% commanded, that's M1 at 78% and M2 at 42% — a big,
// easy-to-see differential.
//
// To protect the geartrain we clamp the final pulse-width delta to the
// calibrated forward/reverse endpoint magnitude for that wheel; boosting
// past the endpoint would over-drive the servo and isn't physically useful
// because the motor is already saturated there.
static inline int applyMotorBias(int us, bool isM1) {
  if (us == 0 || us == MOTOR_STOP || motorBiasPct == 0) return us;
  int mult = 100 + (isM1 ? motorBiasPct : -motorBiasPct);
  if (mult < 0) mult = 0;                // would reverse direction → hold neutral
  int delta  = us - MOTOR_STOP;
  delta = delta * mult / 100;
  int maxMag = abs((isM1 ? M1_FWD : M2_FWD) - MOTOR_STOP);
  if (delta >  maxMag) delta =  maxMag;
  if (delta < -maxMag) delta = -maxMag;
  return MOTOR_STOP + delta;
}

void updateServos() {
  static unsigned long lastPulse = 0;
  if (micros() - lastPulse < 20000) return;
  lastPulse = micros();
  int m1Tgt = applyMotorBias(motor1_us, true);
  int m2Tgt = applyMotorBias(motor2_us, false);
  motor1_us_out = slewToward(motor1_us_out, m1Tgt);
  motor2_us_out = slewToward(motor2_us_out, m2Tgt);
  taskENTER_CRITICAL(&pwmMux);
  if (motor1_us_out > 0) { digitalWrite(MOTOR1_PIN, HIGH); delayMicroseconds(motor1_us_out); digitalWrite(MOTOR1_PIN, LOW); }
  if (motor2_us_out > 0) { digitalWrite(MOTOR2_PIN, HIGH); delayMicroseconds(motor2_us_out); digitalWrite(MOTOR2_PIN, LOW); }
  taskEXIT_CRITICAL(&pwmMux);
}

void motorsOff() { motor1_us = 0; motor2_us = 0; }

int speedToUs(int pct, int fwdUs) {
  if (pct == 0) return 0;
  return MOTOR_STOP + (fwdUs - MOTOR_STOP) * pct / 100;
}

int scaledM1() { return MOTOR_STOP + (M1_FWD - MOTOR_STOP) * driveSpeedPct / 100; }
int scaledM2() { return MOTOR_STOP + (M2_FWD - MOTOR_STOP) * driveSpeedPct / 100; }

void driveForward() { motor1_us = scaledM1(); motor2_us = scaledM2(); }

void forwardWithTrim(float drift) {
  int m1 = scaledM1(), m2 = scaledM2();
  int trim = constrain((int)(fabs(drift) * TRIM_GAIN), 0, TRIM_MAX);
  if (drift > DEADBAND_DEG) {
    motor1_us = constrain(m1 - trim, MOTOR_STOP, m1);
    motor2_us = m2;
  } else if (drift < -DEADBAND_DEG) {
    motor1_us = m1;
    motor2_us = constrain(m2 + trim, m2, MOTOR_STOP);
  } else {
    motor1_us = m1; motor2_us = m2;
  }
}

// Spin in place: M1=left wheel, M2=right wheel for Emitter
void spinToward(float drift) {
  if (drift > 0) { motor1_us = 0; motor2_us = scaledM2(); }
  else            { motor1_us = scaledM1(); motor2_us = 0; }
}

// Drive backward at scaled speed (both motors reversed)
void driveBackward() {
  motor1_us = MOTOR_STOP + (MOTOR_STOP - M1_FWD) * driveSpeedPct / 100;
  motor2_us = MOTOR_STOP + (MOTOR_STOP - M2_FWD) * driveSpeedPct / 100;
}

// Signed turn-in-place: +rate = CCW (left), -rate = CW (right), magnitude 0-100.
// Counter-rotation: both wheels driven in opposite directions for a true
// spin-in-place (was pivot: one wheel off, one driven). Halves the turn
// radius and doubles the angular velocity at the same `rate` value.
void turnInPlace(int8_t rate) {
  if (rate == 0) { motorsOff(); return; }
  int mag = rate > 0 ? rate : -rate;
  if (rate > 0) {
    // CCW: M1 reverse, M2 forward (M1_FWD is above STOP, M2_FWD is below)
    motor1_us = MOTOR_STOP - (M1_FWD - MOTOR_STOP) * mag / 100;
    motor2_us = MOTOR_STOP + (M2_FWD - MOTOR_STOP) * mag / 100;
  } else {
    // CW: M1 forward, M2 reverse
    motor1_us = MOTOR_STOP + (M1_FWD - MOTOR_STOP) * mag / 100;
    motor2_us = MOTOR_STOP - (M2_FWD - MOTOR_STOP) * mag / 100;
  }
}

void triggerBump(Phase resumePhase, int preferDir) {
  bumpState = BUMP_BACKING;
  bumpEndTime = millis() + 800;
  bumpDir = preferDir != 0 ? preferDir : (random(2) ? 1 : -1);
  bumpReturnPhase = resumePhase;
  motorsOff();
  stallPrevTime = 0;
  calTracking = false;
  seekProbing = false;       // invalidate any in-flight probe — baseline is now stale
  SERIAL_LOG.printf("BUMP recovery: back 800ms, turn 270° %s\n", bumpDir > 0 ? "CCW" : "CW");
}

// Returns true if bump recovery is currently running (caller should skip normal drive logic).
bool handleBumpRecovery() {
  if (bumpState == BUMP_IDLE) return false;
  unsigned long now = millis();

  if (bumpState == BUMP_BACKING) {
    if (now < bumpEndTime) { driveBackward(); return true; }
    bumpState = BUMP_TURNING;
    bumpEndTime = now + 3500;
    bumpPrevHdg = readHeading();
    bumpCumTurn = 0;
  }

  if (bumpState == BUMP_TURNING) {
    if (imuOK) {
      float cur = readHeading();
      bumpCumTurn += headingDiff(bumpPrevHdg, cur);
      bumpPrevHdg = cur;
    }
    bool doneByAngle = imuOK && fabs(bumpCumTurn) >= 270.0f;
    bool doneByTime  = now >= bumpEndTime;
    if (doneByAngle || doneByTime) {
      bumpState = BUMP_IDLE;
      motorsOff();
      SERIAL_LOG.printf("BUMP done (turn=%+.0f°)\n", bumpCumTurn);
      return false;
    }
    turnInPlace(bumpDir > 0 ? driveSpeedPct : -driveSpeedPct);
    return true;
  }
  return false;
}

// Accelerometer-based impact detection. Was previously disabled due to false
// triggers from motor PWM startup; tuned thresholds (ACCEL_IMPACT_MPS2 +
// DRIVE_BLANK_MS) and reliance on direction-agnostic motion gating make it
// usable as the sole collision detector when LEAN_MODE suspends UWB (and
// therefore position-based stall detection).
#define ENABLE_ACCEL_BUMP 0

// True when either motor is commanded to any non-trivial speed (forward or
// reverse). Used to gate accel-bump checks during performance playback,
// where sequences may drive reverse or spin-in-place.
inline bool isMotorCommanded() {
  int m1Off = motor1_us > 0 ? abs(motor1_us - MOTOR_STOP) : 0;
  int m2Off = motor2_us > 0 ? abs(motor2_us - MOTOR_STOP) : 0;
  return (m1Off > 50) || (m2Off > 50);
}

void checkAccelBump(Phase currentPhase) {
#if ENABLE_ACCEL_BUMP
  if (!imuOK || !bno) return;
  unsigned long now = millis();
  if (now - lastAccelCheck < ACCEL_CHECK_MS) return;
  lastAccelCheck = now;
  if (!isMotorCommanded()) { drivingSince = 0; return; }
  if (drivingSince == 0) { drivingSince = now; return; }
  if (now - drivingSince < DRIVE_BLANK_MS) return;
  imu::Vector<3> la = bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float mag = sqrtf(la.x()*la.x() + la.y()*la.y() + la.z()*la.z());
  if (mag > ACCEL_IMPACT_MPS2) {
    SERIAL_LOG.printf("IMPACT |a|=%.2f m/s² → bump\n", mag);
    triggerBump(currentPhase, 0);
    drivingSince = 0;
  }
#else
  (void)currentPhase;
#endif
}

// Distance from a point to a line segment (all values in cm, anchor-frame coords).
float distPointToSegment(float px, float py, float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  float lenSq = dx * dx + dy * dy;
  if (lenSq < 0.01f) {
    float ddx = px - x1, ddy = py - y1;
    return sqrtf(ddx * ddx + ddy * ddy);
  }
  float t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  float cx = x1 + t * dx, cy = y1 + t * dy;
  float ddx = px - cx, ddy = py - cy;
  return sqrtf(ddx * ddx + ddy * ddy);
}

// Check drawn no-go obstacle: line segment on the map. Bump if we get too close.
void checkNoGo(Phase currentPhase) {
  if (noGoX1 == 0 && noGoY1 == 0 && noGoX2 == 0 && noGoY2 == 0) return;  // disabled
  if (!posValid) return;
  unsigned long now = millis();
  if (now - lastNoGoTrig < NOGO_COOLDOWN_MS) return;

  float d = distPointToSegment(tagX, tagY, noGoX1, noGoY1, noGoX2, noGoY2);
  if (d < NOGO_THRESH_CM) {
    SERIAL_LOG.printf("NO-GO: %.0fcm from obstacle (%d,%d)-(%d,%d) → bump\n",
                      d, noGoX1, noGoY1, noGoX2, noGoY2);
    lastNoGoTrig = now;
    triggerBump(currentPhase, 0);
  }
}

// Check if robot is driving forward but not actually moving (physical obstacle).
void checkStall(Phase currentPhase) {
  if (!isDrivingForward() || !posValid) { stallPrevTime = 0; return; }
  unsigned long now = millis();
  if (stallPrevTime == 0) { stallPrevX = tagX; stallPrevY = tagY; stallPrevTime = now; return; }
  if (now - stallPrevTime < STALL_WINDOW_MS) return;
  float dx = tagX - stallPrevX, dy = tagY - stallPrevY;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < STALL_MIN_MOVE_CM) {
    SERIAL_LOG.printf("STALL: only %.0fcm in %lums while driving → bump!\n", d, STALL_WINDOW_MS);
    triggerBump(currentPhase, 0);
  } else {
    stallPrevX = tagX; stallPrevY = tagY; stallPrevTime = now;
  }
}

// Auto-calibration: refine mapBasisHdg from observed motion direction vs compass.
// Only runs while robot is driving mostly-forward and has a valid UWB position.
void maybeAutoCalibrate(float drift) {
  if (!posValid || !imuOK || fabs(drift) > 25.0f) { calTracking = false; return; }
  unsigned long now = millis();
  if (!calTracking) { calPrevX = tagX; calPrevY = tagY; calPrevTime = now; calTracking = true; return; }
  if (now - calPrevTime < 800) return;   // was 1200 — faster convergence
  float dx = tagX - calPrevX, dy = tagY - calPrevY;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 10.0f) {                        // was 12 — shorter motion window (more updates)
    if (now - calPrevTime > 5000) { calPrevX = tagX; calPrevY = tagY; calPrevTime = now; }
    return;
  }
  // BNO055 heading is CCW-positive (Bosch datasheet: heading INCREASES with CCW
  // rotation when viewed from above). atan2(dy,dx) is also CCW-positive from +X.
  // Invariant: compass = mapBasisHdg + mapAngle  →  mapBasisHdg = compass - mapAngle.
  float mapAngle = atan2f(dy, dx) * 180.0f / M_PI;
  float compass = readHeading();
  float apparent = fmodf(compass - mapAngle + 360.0f, 360.0f);
  float diff = headingDiff(mapBasisHdg, apparent);
  // 0.7 blend (was 0.55): each cycle collapses ~70% of remaining error → 2 cycles to 9%, 3 to 3%.
  mapBasisHdg = fmodf(mapBasisHdg + diff * 0.70f + 360.0f, 360.0f);
  SERIAL_LOG.printf("AUTOCAL d=%.0f mapAng=%+.0f comp=%.0f basis=>%.0f (corr=%+.1f)\n",
                    d, mapAngle, compass, mapBasisHdg, diff * 0.70f);
  calPrevX = tagX; calPrevY = tagY; calPrevTime = now;
  seekWrongCount = 0;  // moved → believe auto-cal, reset wrong-way counter
}

// ── IMU ──────────────────────────────────────────────────

float readHeading() {
  if (!imuOK) return 0;
  sensors_event_t e;
  bno->getEvent(&e);
  return e.orientation.x;
}

float headingDiff(float from, float to) {
  float d = to - from;
  while (d >  180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// ── Matrix ───────────────────────────────────────────────

void matrixFull() { if (!lc) return; for (int r = 0; r < 8; r++) lc->setRow(0, r, 0xFF); }
void matrixOff()  { if (!lc) return; for (int r = 0; r < 8; r++) lc->setRow(0, r, 0x00); }

void showArrow(int dir) {
  if (!lc || dir < 0 || dir > 7) return;
  for (int r = 0; r < 8; r++)
    lc->setRow(0, r, arrows[dir][r]);
}

// ── Matrix Animation: mode-driven state machine ────────
// The matrix is always in exactly one of these modes. Modes switch only
// when a new 0x07 command arrives, when the autoplay anim track runs an
// SOP_ANIM_* step, or when a mode naturally terminates (e.g. OVOID_MERGE
// → OVOID_HOLD once the merge completes).
//
//   OFF         Blank. No frames drawn. Matrix register holds zeros.
//   BOUNCE      N bouncing pixels, N = matrixComplexity (1-5). One pixel
//               per complexity step. Pixels reflect off walls and randomise
//               direction every 1-3 s.
//   GOL         5 bouncing pixels seed a life buffer; GoL rules evolve the
//               life buffer each frame. Higher complexity (6-10) adds
//               trails, trail decay, and noise injection.
//   OVOID_MERGE Probabilistic merge of the current display into the fixed
//               6-pixel ovoid pattern: every frame, ovoid cells are forced
//               ON and non-ovoid cells have a 1/4 chance of turning OFF.
//               Transitions to OVOID_HOLD when nothing but the ovoid
//               remains lit.
//   OVOID_HOLD  Static 6-pixel ovoid. No frames drawn — the MAX7219 latches
//               the bits; the mode exists just to know we're done merging.
//
// `matrixComplexity` is the per-mode detail knob (bouncer count for BOUNCE,
// GoL-trail depth for GOL). `matrixSpeedMs` is ms/frame for every mode.

enum MatrixMode : uint8_t { MMODE_OFF, MMODE_BOUNCE, MMODE_GOL,
                            MMODE_OVOID_MERGE, MMODE_OVOID_HOLD };

uint8_t       matrixMode       = MMODE_OFF;
uint8_t       matrixComplexity = 0;
unsigned long matrixSpeedMs    = 150;  // ms per animation frame
unsigned long lastMatrixFrame  = 0;

// Minimal ovoid: six lit pixels — one cap top and bottom, two stacked on
// each side at the equator. Total = 1 + 2 + 2 + 1 = 6 lit cells. Caps sit
// at column 3 (slightly left of true centre on the 8-wide grid).
static const uint8_t MATRIX_OVOID_ROWS[8] = {
  0b00010000,  // . . . X . . . .   top cap
  0b00000000,
  0b00000000,
  0b10000001,  // X . . . . . . X   sides, row 1
  0b10000001,  // X . . . . . . X   sides, row 2
  0b00000000,
  0b00000000,
  0b00010000,  // . . . X . . . .   bottom cap
};

struct Bouncer { int x, y, dx, dy; unsigned long nextNudgeT; };
Bouncer bouncers[5];
uint8_t lifeBuf[8];
uint8_t dispBuf[8];     // current on-screen state — retained across frames
                        // so mode switches (especially → OVOID_MERGE) can
                        // start from whatever is currently shown.

static unsigned long randNudgeDelay() {
  return 1000UL + (unsigned long)random(2001);   // 1000–3000 ms
}

// Pick a fresh 8-way direction for bouncer i that isn't a full stop.
// Inlined (instead of a helper) so Arduino's auto-generated prototypes don't
// need to see the Bouncer struct before it's declared.
#define RANDOMIZE_DIR(i) do { \
    do { bouncers[i].dx = (int)random(3) - 1; \
         bouncers[i].dy = (int)random(3) - 1; \
    } while (bouncers[i].dx == 0 && bouncers[i].dy == 0); \
  } while (0)

// Reseed the bouncer positions (called when entering BOUNCE or GOL). Keeps
// dispBuf alone — we WANT dispBuf to retain whatever the previous mode
// left on screen so a mode swap doesn't blank the matrix for one frame.
void matrixAnimInit() {
  memset(lifeBuf, 0, 8);
  unsigned long now = millis();
  for (int i = 0; i < 5; i++) {
    bouncers[i].x = (int)random(8);
    bouncers[i].y = (int)random(8);
    RANDOMIZE_DIR(i);
    bouncers[i].nextNudgeT = now + randNudgeDelay();
  }
  lastMatrixFrame = 0;
}

// Apply one mode change. Called from both the 0x07 BLE handler and the
// SOP_ANIM_* autoplay ops; kept in one place so the two paths can't drift.
// speedByte is the raw firmware param (5-50) in 10ms units; pass the same
// value the live slider / step editor shows.
void matrixSetMode(uint8_t mode, uint8_t cmplx, uint8_t speedByte) {
  matrixSpeedMs = (unsigned long)constrain((int)speedByte, 5, 50) * 10;
  switch (mode) {
    case MMODE_OFF:
      matrixMode = MMODE_OFF;
      matrixComplexity = 0;
      memset(dispBuf, 0, 8);
      if (lc) for (int r = 0; r < 8; r++) lc->setRow(0, r, 0x00);
      break;
    case MMODE_BOUNCE:
      matrixMode = MMODE_BOUNCE;
      matrixComplexity = constrain((int)cmplx, 1, 5);
      matrixAnimInit();
      break;
    case MMODE_GOL:
      matrixMode = MMODE_GOL;
      matrixComplexity = constrain((int)cmplx, 6, 10);
      matrixAnimInit();
      break;
    case MMODE_OVOID_MERGE:
      // Leave dispBuf as-is: the merge animation transforms the current
      // on-screen state into the ovoid pixel-by-pixel. If we're jumping
      // straight to ovoid from OFF, dispBuf is zero and the merge draws
      // the ovoid one cell at a time anyway.
      matrixMode = MMODE_OVOID_MERGE;
      matrixComplexity = 0;
      break;
    default:
      break;
  }
}

// One animation frame. Dispatches by current mode; each mode produces the
// next dispBuf[] and we then push it to the MAX7219 in one go. Called from
// loop() at line rate; internal throttling limits actual frame updates to
// matrixSpeedMs.
void matrixAnimTick() {
  if (!lc) return;
  if (matrixMode == MMODE_OFF || matrixMode == MMODE_OVOID_HOLD) return;
  if (matrixSpeedMs == 0) return;

  const unsigned long now = millis();
  if (now - lastMatrixFrame < matrixSpeedMs) return;
  lastMatrixFrame = now;

  if (matrixMode == MMODE_BOUNCE || matrixMode == MMODE_GOL) {
    int nb = (matrixMode == MMODE_BOUNCE)
             ? constrain((int)matrixComplexity, 1, 5)
             : 5;   // GOL always seeds from 5 bouncers

    // Continuous bounce: step each bouncer one cell per frame in its
    // current direction, reflect off walls. Every 1-3 s per bouncer,
    // randomise direction so motion doesn't settle into a loop.
    for (int i = 0; i < nb; i++) {
      if ((long)(now - bouncers[i].nextNudgeT) >= 0) {
        RANDOMIZE_DIR(i);
        bouncers[i].nextNudgeT = now + randNudgeDelay();
      }
      bouncers[i].x += bouncers[i].dx;
      bouncers[i].y += bouncers[i].dy;
      if (bouncers[i].x >= 7) { bouncers[i].x = 7; bouncers[i].dx = -abs(bouncers[i].dx); if (bouncers[i].dx == 0) bouncers[i].dx = -1; }
      if (bouncers[i].x <= 0) { bouncers[i].x = 0; bouncers[i].dx =  abs(bouncers[i].dx); if (bouncers[i].dx == 0) bouncers[i].dx =  1; }
      if (bouncers[i].y >= 7) { bouncers[i].y = 7; bouncers[i].dy = -abs(bouncers[i].dy); if (bouncers[i].dy == 0) bouncers[i].dy = -1; }
      if (bouncers[i].y <= 0) { bouncers[i].y = 0; bouncers[i].dy =  abs(bouncers[i].dy); if (bouncers[i].dy == 0) bouncers[i].dy =  1; }
    }

    if (matrixMode == MMODE_GOL) {
      // Seed life buffer with current bouncer positions, then evolve.
      for (int i = 0; i < nb; i++)
        lifeBuf[bouncers[i].y] |= (1 << bouncers[i].x);

      if (matrixComplexity >= 7) {
        // Full GoL evolution on the trail buffer.
        uint8_t next[8];
        for (int y = 0; y < 8; y++) {
          next[y] = 0;
          for (int x = 0; x < 8; x++) {
            int n = 0;
            for (int dy = -1; dy <= 1; dy++)
              for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                if (lifeBuf[(y+dy+8)%8] & (1 << ((x+dx+8)%8))) n++;
              }
            bool alive = lifeBuf[y] & (1 << x);
            if (alive && (n == 2 || n == 3)) next[y] |= (1 << x);
            else if (!alive && n == 3)       next[y] |= (1 << x);
          }
        }
        bool allDead = true;
        for (int i = 0; i < 8; i++) if (next[i]) { allDead = false; break; }
        if (allDead) for (int i = 0; i < 8; i++) next[i] = random(256);
        memcpy(lifeBuf, next, 8);
        if (matrixComplexity >= 9) {
          int chance = (matrixComplexity >= 10) ? 2 : 4;
          if (random(chance) == 0) lifeBuf[random(8)] |= (1 << random(8));
        }
      } else {
        // complexity 6: trail mode — decay cells slowly, no GoL yet.
        for (int y = 0; y < 8; y++)
          for (int x = 0; x < 8; x++)
            if ((lifeBuf[y] & (1 << x)) && random(8) == 0)
              lifeBuf[y] &= ~(1 << x);
      }
    }

    memset(dispBuf, 0, 8);
    for (int i = 0; i < nb; i++)
      dispBuf[bouncers[i].y] |= (1 << bouncers[i].x);
    if (matrixMode == MMODE_GOL)
      for (int y = 0; y < 8; y++) dispBuf[y] |= lifeBuf[y];

  } else if (matrixMode == MMODE_OVOID_MERGE) {
    // Merge whatever dispBuf currently holds into the ovoid: force every
    // ovoid cell ON, and give every non-ovoid lit cell a 1-in-4 chance of
    // turning OFF this frame. Once nothing lit remains outside the ovoid
    // shape, switch to OVOID_HOLD (we're done — no more frames needed).
    bool anyExtra = false;
    for (int y = 0; y < 8; y++) {
      uint8_t target = MATRIX_OVOID_ROWS[y];
      uint8_t cur    = dispBuf[y];
      uint8_t extras = cur & ~target;
      if (extras) {
        for (int x = 0; x < 8; x++) {
          if ((extras & (1 << x)) && random(4) == 0) {
            cur &= ~(1 << x);
            extras &= ~(1 << x);
          }
        }
        if (extras) anyExtra = true;
      }
      dispBuf[y] = cur | target;   // always force ovoid cells ON
    }
    if (!anyExtra) matrixMode = MMODE_OVOID_HOLD;
  }

  for (int r = 0; r < 8; r++) lc->setRow(0, r, dispBuf[r]);
}

// ── BLE ──────────────────────────────────────────────────

class CmdCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String v = pChar->getValue();
    cmdLen = min((int)v.length(), 12);
    for (int i = 0; i < cmdLen; i++) cmdBuf[i] = (uint8_t)v[i];
    if (cmdLen > 0) cmdPending = true;
  }
};

// Forward decls for the autoplay state accessors used below. The bodies
// live next to the autoplay state (inside the #if AUTOPLAY block) with
// #else stubs so this compiles cleanly with AUTOPLAY=0 too.
uint8_t  autoplayMotorIdxByte();
uint8_t  autoplayAnimIdxByte();
uint16_t autoplayMotorMsLeft();

// stop()/start() is required around setAdvertisementData() on this Arduino-ESP32 BLE
// stack — updating advertising data live sometimes doesn't actually publish the new
// payload (especially when manufacturer-data length changes). The old delay(10) is
// gone, and the 1 Hz throttle in checkBleEvents() keeps the radio current low enough
// to avoid brown-outs.
//
// Manufacturer-data layout (14 bytes total, fixed length so in-place update works):
//   [0..1]   0xFF 0xFF                    — manufacturer id (test/dev)
//   [2..3]   d0 (BE)                      — UWB anchor 0 distance, 0 in LEAN_MODE
//   [4..5]   d1 (BE)                      — UWB anchor 1 distance, 0 in LEAN_MODE
//   [6]      status                       — phase, imuOK, kfInit, tid
//   [7]      heading                      — yaw / 2 (0..179 → 0..358°)
//   [8]      motor next-idx (0xFF = idle) — next step the runner will dispatch
//   [9]      anim  next-idx (0xFF = idle)
//   [10..11] motor ms-until-next (BE)     — clamps at 65535
//   [12..13] free heap / 32 (BE)          — 32-byte resolution, 0..2 MB range
// Bytes 8..11 are read by the receiver (T1) for inter-robot sequence sync.
// Bytes 12..13 are read by the anchor and relayed to the web UI as a
// running heap-health gauge (early warning of any future BLE leak).
// T0 (this robot) is the leader and ignores the peer's matching bytes.
void pushBle(const char* reason) {
  if (!pBleAdv) return;

  uint16_t d0 = kfAnc[0].init ? (uint16_t)constrain((int)kfAnc[0].x, 0, 65535) : 0;
  uint16_t d1 = kfAnc[1].init ? (uint16_t)constrain((int)kfAnc[1].x, 0, 65535) : 0;
  // Bit 7 encodes tag id (0 = Emitter/T0, 1 = Receiver/T1). The anchor reads
  // this out of the primary advertisement manufacturer data, so it never has
  // to depend on the BLE name (which arrives in the SCAN_RSP and can be
  // missed or cross-barred between two simultaneously-advertising robots).
  uint8_t status = ((uint8_t)phase & 0x07)
                 | (imuOK ? 0x08 : 0)
                 | (kfInit ? 0x10 : 0)
                 | 0x00;   // tid = 0
  uint8_t hdg = (uint8_t)(readHeading() / 2.0);

  uint8_t  syncMotor = autoplayMotorIdxByte();
  uint8_t  syncAnim  = autoplayAnimIdxByte();
  uint16_t syncMs    = autoplayMotorMsLeft();

  // Heap in 32-byte units, clamped to 16 bits. ESP32-S3 typically reports
  // ~150-200 KB free with BLE up; 32-byte resolution catches drift well
  // below the noise floor of normal allocator activity.
  uint32_t freeHeap   = ESP.getFreeHeap();
  uint16_t heapUnits  = (uint16_t)constrain((int)(freeHeap >> 5), 0, 65535);

  BLEAdvertisementData ad;
  ad.setName("RobotA-Tx");
  ad.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  uint8_t mf[14] = { 0xFF, 0xFF,
    (uint8_t)(d0 >> 8), (uint8_t)(d0 & 0xFF),
    (uint8_t)(d1 >> 8), (uint8_t)(d1 & 0xFF),
    status, hdg,
    syncMotor, syncAnim,
    (uint8_t)(syncMs >> 8),     (uint8_t)(syncMs & 0xFF),
    (uint8_t)(heapUnits >> 8),  (uint8_t)(heapUnits & 0xFF) };
  ad.setManufacturerData(String((char*)mf, 14));

  // In-place update: avoids the stop/delay/start radio transient (big current spike
  // that can coincide with motor peaks → brownout). Length is fixed at 12 bytes so
  // the stack accepts this without a restart.
  pBleAdv->setAdvertisementData(ad);

  lastBlePush  = millis();
  lastBleDist  = (uint8_t)constrain((int)d0, 0, 255);
  lastBlePhase = (uint8_t)phase;

  SERIAL_LOG.printf("BLE>> %s  d0=%d d1=%d phase=%d  motor=%u anim=%u ms=%u\n",
                    reason, d0, d1, (int)phase,
                    (unsigned)syncMotor, (unsigned)syncAnim, (unsigned)syncMs);
}

// Rate-limit non-critical updates; phase changes still push immediately.
// 500 ms = 2 Hz default broadcast rate. DIST-threshold pushes still trigger
// on motion within this cadence. In-place setAdvertisementData (no radio
// stop/restart) keeps the per-broadcast current spike small, so 2 Hz should
// coexist with motor load — if brownouts re-appear under movement, bump
// this to 750 or 1000.
const unsigned long BLE_MIN_INTERVAL_MS = 500;

void checkBleEvents() {
  uint8_t d0now = kfAnc[0].init ? (uint8_t)constrain((int)kfAnc[0].x, 0, 255) : 0;
  unsigned long sinceLast = millis() - lastBlePush;
  if ((uint8_t)phase != lastBlePhase) { pushBle("PHASE"); return; }
  if (sinceLast < BLE_MIN_INTERVAL_MS) return;
  if (abs((int)d0now - (int)lastBleDist) >= BLE_DIST_THRESHOLD) { pushBle("DIST"); return; }
  if (sinceLast >= BLE_HEARTBEAT_MS) { pushBle("HB"); }
}

// Drop the BLE scan's accumulated results vector periodically. The
// Arduino-ESP32 BLE library appends a copy of every received advertisement
// (the `true` second arg to setAdvertisedDeviceCallbacks asks for
// duplicates too) to an internal vector that is *never* auto-cleared while
// a duration=0 (forever) scan is active. With ambient BLE traffic + the
// peer robot advertising at ~2 Hz, the heap fragments to death in ~2 min
// and the BLE controller hard-faults the chip. Since PeerScanCB handles
// every ad we care about live, the cached results are pure leak — we just
// flush them.
unsigned long lastScanFlush = 0;
const unsigned long SCAN_FLUSH_MS = 10000;
// Tracks the lowest free-heap value we've ever seen at flush time. If the
// fix is holding, this value should plateau within the first minute and
// stop dropping. Continuous downward drift = the leak is back.
size_t heapLowWater = SIZE_MAX;
void flushScanResults() {
  if (!pBleScan) return;
  unsigned long now = millis();
  if (now - lastScanFlush < SCAN_FLUSH_MS) return;
  lastScanFlush = now;
  pBleScan->clearResults();
  size_t freeNow = ESP.getFreeHeap();
  if (freeNow < heapLowWater) heapLowWater = freeNow;
  SERIAL_LOG.printf("HEAP: free=%u min_ever=%u low_water=%u uptime=%lus\n",
                    (unsigned)freeNow,
                    (unsigned)ESP.getMinFreeHeap(),
                    (unsigned)heapLowWater,
                    now / 1000UL);
}

// ── Autoplay: hard-coded sequence table + runner ─────────
// The actual step data lives in the sidecar file autoplay_sequences.h,
// which is regenerated by the browser's "Export fw" button and dropped
// into the sketch folder (same file on both robots). Editing sequences
// happens in the browser — this .ino is only the runner.
//
// NOTE: the enum and struct are declared *outside* the #if AUTOPLAY gate
// so Arduino IDE's auto-prototype generator (which runs on every function
// in a .ino before any type declarations it hasn't seen yet) can resolve
// references to them in the runner's forward prototypes.
//
// Autoplay ops. Motor ops live on the motor track; SOP_ANIM_* + SOP_LED
// live on the anim track. Both tracks run in parallel on their own
// cursors. Motor track is primary: when it reaches SOP_END the whole
// sequence finishes and the anim track aborts.
//
// p1/p2 meanings:
//   SOP_FWD/REV     : p1 = speed %,      p2 = 0
//   SOP_TL/TR       : p1 = turn speed %, p2 = 0
//   SOP_PAUSE       : p1 = 0,            p2 = 0
//   SOP_WALK        : p1 = speed %,      p2 = 0
//   SOP_ANIM_BOUNCE : p1 = cmplx (1-5),  p2 = anim speed (5-50, 10ms units)
//   SOP_ANIM_GOL    : p1 = cmplx (6-10), p2 = anim speed
//   SOP_ANIM_OVOID  : p1 = 0,            p2 = anim speed
//   SOP_ANIM_OFF    : p1 = 0,            p2 = 0
//   SOP_LED         : p1 = mode (0..3),  p2 = 0
// Motor bias is NOT a per-step op — it's a global per-robot setting
// controlled by the Manual-Drive slider (0x11 command) and persisted on
// each robot independently.
enum AutoOp : uint8_t {
  SOP_END = 0, SOP_FWD, SOP_REV, SOP_TL, SOP_TR, SOP_PAUSE, SOP_WALK,
  SOP_ANIM_BOUNCE, SOP_ANIM_GOL, SOP_ANIM_OVOID, SOP_ANIM_OFF,
  SOP_LED,
};

// AutoStep layout: { op, p1, p2, target, dur_ms } — 8 bytes, no padding.
//   target: 0=T0 only, 1=T1 only, 0xFF=Both. Steps not matching MY_ROBOT_ID
//           and not 0xFF are skipped with no wait, so per-target step
//           durations don't bleed across robots (matches the browser
//           runner's per-target-timeline semantics).
//   dur_ms: uint32_t so a single step can run for minutes (e.g. a long
//           SOP_ANIM_GOL that holds for the entire motor track). Was
//           uint16_t and tripped a -Wnarrowing error on long-duration export.
struct AutoStep { uint8_t op; uint8_t p1; uint8_t p2; uint8_t target; uint32_t dur_ms; };
// AutoSeq pairs the two tracks of a sequence. Kept here (not in the
// generated header) so the struct lives next to AutoStep and AutoOp. The
// header just instantiates AUTOPLAY_TABLE[] using this type.
struct AutoSeq  { const AutoStep* motor; const AutoStep* anim; };

#if AUTOPLAY

// Pulls in SEQ_*_MOTOR[], SEQ_*_ANIM[], AUTOPLAY_TABLE[] (array of AutoSeq),
// AUTOPLAY_NAMES[], and AUTOPLAY_COUNT. Regenerate from
// position_map/index.html (Test Sequence card → "Export fw").
#include "autoplay_sequences.h"

// ── Motor track runner ────────────────────────────────
// The motor track is the "primary" track: its SOP_END triggers repeat
// logic and, on the final repeat, ends the whole autoplay session.
unsigned long   autoplayStartAt    = 0;      // millis() deadline to begin first step
bool            autoplayActive     = false;
const AutoStep* autoMotor          = nullptr;
uint16_t        autoMotorIdx       = 0;
unsigned long   autoMotorStepEnd   = 0;
uint16_t        autoRepeatDone     = 0;

// ── Anim track runner ─────────────────────────────────
// Independent cursor over the animation track. Advances on its own
// timers; stops whenever the motor track ends the sequence.
const AutoStep* autoAnim           = nullptr;
uint16_t        autoAnimIdx        = 0;
unsigned long   autoAnimStepEnd    = 0;

// State accessors used by pushBle() to publish runner state for inter-
// robot sequence sync. Returns 0xFF when the corresponding track is idle
// (not started, finished, or not loaded). uint16_t ms-left is clamped to
// 65535. Indices > 254 also clamp to 254 so we keep 0xFF reserved for
// "idle" — sequences that long would never fit in the export anyway.
uint8_t  autoplayMotorIdxByte() {
  if (!autoplayActive || !autoMotor) return 0xFF;
  return (uint8_t)(autoMotorIdx > 254 ? 254 : autoMotorIdx);
}
uint8_t  autoplayAnimIdxByte() {
  if (!autoplayActive || !autoAnim) return 0xFF;
  return (uint8_t)(autoAnimIdx > 254 ? 254 : autoAnimIdx);
}
uint16_t autoplayMotorMsLeft() {
  if (!autoplayActive) return 0;
  unsigned long now = millis();
  if (autoMotorStepEnd <= now) return 0;
  unsigned long left = autoMotorStepEnd - now;
  return left > 65535 ? 65535 : (uint16_t)left;
}

void autoplayBegin(int seqIdx) {
  if (seqIdx < 1 || seqIdx > AUTOPLAY_COUNT) {
    SERIAL_LOG.printf("AUTOPLAY: invalid sequence %d — skipping\n", seqIdx);
    return;
  }
  const AutoSeq &s = AUTOPLAY_TABLE[seqIdx];
  if (!s.motor || !s.anim) return;
  autoMotor        = s.motor;
  autoAnim         = s.anim;
  autoMotorIdx     = 0;
  autoAnimIdx      = 0;
  autoMotorStepEnd = 0;
  autoAnimStepEnd  = 0;
  autoRepeatDone   = 0;
  autoplayActive   = true;
  phase = PH_MANUAL;
  motorsOff(); walkActive = false;
  SERIAL_LOG.printf("AUTOPLAY begin: seq=%d \"%s\" repeat=%d\n",
                    seqIdx, AUTOPLAY_NAMES[seqIdx], AUTOPLAY_REPEAT);
}

void autoplayTick() {
  if (!autoplayActive) return;
  unsigned long now = millis();

  // ── Motor track ──
  // Loop so a chain of "not-for-me" steps collapses into one tick.
  // Steps targeting the OTHER robot are skipped with no wait (their
  // dur_ms doesn't bleed onto our timeline). SOP_END always applies
  // to everyone regardless of its target byte.
  while (autoMotor && now >= autoMotorStepEnd) {
    uint8_t  op     = autoMotor[autoMotorIdx].op;
    uint8_t  p1     = autoMotor[autoMotorIdx].p1;
    uint8_t  target = autoMotor[autoMotorIdx].target;
    uint32_t dur_ms = autoMotor[autoMotorIdx].dur_ms;

    if (op == SOP_END) {
      autoRepeatDone++;
      // 0 = loop forever; 1 = once; N = N times total.
      bool keepGoing = (AUTOPLAY_REPEAT == 0) || (autoRepeatDone < AUTOPLAY_REPEAT);
      if (keepGoing) {
        SERIAL_LOG.printf("AUTOPLAY repeat %u\n", (unsigned)autoRepeatDone + 1);
        autoMotorIdx     = 0;
        autoAnimIdx      = 0;       // rewind anim in lockstep
        autoAnimStepEnd  = 0;
        motorsOff(); walkActive = false;
        autoMotorStepEnd = now + 150;  // brief gap between passes
        return;
      }
      motorsOff(); walkActive = false;
      autoplayActive = false;
      SERIAL_LOG.println("AUTOPLAY complete");
      return;
    }

    // Skip steps not for this robot. No wait — advance immediately so
    // T0 doesn't sit idle through T1-only steps and vice versa.
    if (target != MY_ROBOT_ID && target != 0xFF) {
      autoMotorIdx++;
      continue;
    }

    walkActive = false;  // motor ops are one-shot set-and-hold
    switch (op) {
      case SOP_FWD:
        motor1_us = speedToUs((int)p1, M1_FWD);
        motor2_us = speedToUs((int)p1, M2_FWD);
        break;
      case SOP_REV:
        motor1_us = speedToUs(-(int)p1, M1_FWD);
        motor2_us = speedToUs(-(int)p1, M2_FWD);
        break;
      case SOP_TL: turnInPlace((int8_t)p1);  break;
      case SOP_TR: turnInPlace(-(int8_t)p1); break;
      case SOP_PAUSE: motorsOff(); break;
      case SOP_WALK:
        driveSpeedPct = (int)p1;
        walkActive    = true;
        walkM1Turn    = true;
        walkStepStart = millis();
        break;
      default: motorsOff(); break;
    }
    autoMotorStepEnd = now + dur_ms;
    autoMotorIdx++;
    break;  // dispatched — wait for this step to finish before the next
  }

  // ── Anim track ──
  // Runs until it hits SOP_END, then waits silently until the motor
  // track finishes the whole sequence. This is how anim-empty sequences
  // (just SOP_END) correctly do nothing without stalling the runner.
  // Same target-skip semantics as the motor track above.
  while (autoAnim && now >= autoAnimStepEnd) {
    uint8_t  op     = autoAnim[autoAnimIdx].op;
    uint8_t  p1     = autoAnim[autoAnimIdx].p1;
    uint8_t  p2     = autoAnim[autoAnimIdx].p2;
    uint8_t  target = autoAnim[autoAnimIdx].target;
    uint32_t dur_ms = autoAnim[autoAnimIdx].dur_ms;

    if (op == SOP_END) {
      autoAnim = nullptr;  // track done; motor track drives completion
      return;
    }
    if (target != MY_ROBOT_ID && target != 0xFF) {
      autoAnimIdx++;
      continue;
    }
    switch (op) {
      case SOP_ANIM_BOUNCE: matrixSetMode(MMODE_BOUNCE,      p1, p2); break;
      case SOP_ANIM_GOL:    matrixSetMode(MMODE_GOL,         p1, p2); break;
      case SOP_ANIM_OVOID:  matrixSetMode(MMODE_OVOID_MERGE,  0, p2); break;
      case SOP_ANIM_OFF:    matrixSetMode(MMODE_OFF,          0,  0); break;
      case SOP_LED:
        // Emitter has no indicator LED — receiver-only. Ignored silently
        // so a single shared anim track can target both robots at once.
        break;
      default: break;
    }
    autoAnimStepEnd = now + dur_ms;
    autoAnimIdx++;
    break;
  }
}

// Countdown indicator during the AUTOPLAY_DELAY_MS window. Fast blink on
// the matrix so it's obvious the robot is armed but not yet moving. The
// matrix-anim state machine sees MMODE_OFF during this window and stays
// quiet, so the countdown has the matrix to itself.
void autoplayCountdownTick() {
  if (!lc) return;
  static unsigned long lastTick = 0;
  static bool lit = false;
  unsigned long now = millis();
  if (now - lastTick < 250) return;
  lastTick = now;
  lit = !lit;
  if (lit) matrixFull(); else matrixOff();
}

#else
// Stubs so pushBle() compiles when autoplay is off — no runner means
// "idle" on both tracks.
uint8_t  autoplayMotorIdxByte() { return 0xFF; }
uint8_t  autoplayAnimIdxByte()  { return 0xFF; }
uint16_t autoplayMotorMsLeft()  { return 0; }
#endif  // AUTOPLAY

// ── Setup ────────────────────────────────────────────────

void setup() {
  pinMode(RESET_PIN, OUTPUT);
#if LEAN_MODE
  // Hold the DW3000 STM32 in reset so it doesn't come up as a UWB anchor
  // (its flash was configured that way — ~80-100 mA steady, +200 mA
  // bursts). This is the primary brownout mitigation on T0.
  digitalWrite(RESET_PIN, LOW);
#else
  digitalWrite(RESET_PIN, HIGH);
#endif
  pinMode(MOTOR1_PIN, OUTPUT); digitalWrite(MOTOR1_PIN, LOW);
  pinMode(MOTOR2_PIN, OUTPUT); digitalWrite(MOTOR2_PIN, LOW);

  SERIAL_LOG.begin(115200);
#if !LEAN_MODE
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
#endif
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.display();

  SERIAL_LOG.println("=== Robot-A Emitter v4 ===");
  SERIAL_LOG.printf("Trim-only: gain=%.1f max=%d deadband=%.1f\n", TRIM_GAIN, TRIM_MAX, DEADBAND_DEG);

  // MAX7219 — fully lit on startup
  lc = new LedControl(MATRIX_DIN, MATRIX_CLK, MATRIX_CS, 1);
  lc->shutdown(0, false);
  lc->setIntensity(0, 3);  // was 6 — ~40% less LED current, helps brownouts
  matrixFull();

  pinMode(IR_SENSE_PIN, INPUT);

  // DFPlayer init — skip when ENABLE_DFPLAYER=0 to avoid the ~500 ms
  // SD-mount + amp-bias transient on a brownout-prone supply.
#if ENABLE_DFPLAYER
  Serial1.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(500);
  if (dfPlayer.begin(Serial1)) { dfPlayerOK = true; dfPlayer.volume(30); }
  SERIAL_LOG.printf("DFPlayer: %s\n", dfPlayerOK ? "OK" : "FAIL");
#else
  SERIAL_LOG.println("DFPlayer: disabled (ENABLE_DFPLAYER=0)");
#endif

  // BLE + GATT command service.
  // Low TX power (-9 dBm) and low-duty scanning keeps BLE average current down,
  // which avoids the brown-outs seen when scan ran at 50% duty + full power.
  BLEDevice::init("RobotA-Tx");
  BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_DEFAULT);
  BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_SCAN);
  BLEServer* pServer = BLEDevice::createServer();
  BLEService* pSvc = pServer->createService(CMD_SERVICE_UUID);
  BLECharacteristic* pCmdChar = pSvc->createCharacteristic(
    CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCmdChar->setCallbacks(new CmdCallback());
  pSvc->start();
  pBleAdv = BLEDevice::getAdvertising();
  BLEAdvertisementData ad;
  ad.setName("RobotA-Tx");
  ad.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  pBleAdv->setAdvertisementData(ad);
  BLEAdvertisementData scanResp;
  scanResp.setName("RobotA-Tx");
  pBleAdv->setScanResponseData(scanResp);
  pBleAdv->start();
  SERIAL_LOG.printf("BLE: MAC=%s\n", BLEDevice::getAddress().toString().c_str());

  // BLE peer scan (receive Receiver's position while advertising).
  // Old settings: interval=1600/window=80 = 50 ms scan every 1000 ms (5% duty,
  // 1 Hz). With commands changing every ~1 s in sequences, the 1 Hz sampling
  // was too slow: each scan had to randomly align with the anchor's ad
  // packets, causing ~50% command-miss rate.
  //
  // New settings: 10 ms scan every 100 ms = 10% duty, 10 Hz. Peak current is
  // LOWER than the old config (10 ms burst vs 50 ms burst), so brownout risk
  // is not increased. Ten scan windows per second, combined with the anchor
  // now advertising on a tight 20 ms grid, virtually guarantees every
  // command is received within ~100 ms of being sent.
  pBleScan = BLEDevice::getScan();
  pBleScan->setAdvertisedDeviceCallbacks(new PeerScanCB(), true);
  pBleScan->setActiveScan(false);
  pBleScan->setInterval(160);   // 100 ms cycle
  pBleScan->setWindow(16);      //  10 ms window  (10% duty)
  pBleScan->start(0, nullptr, false);
  delay(500);

  SERIAL_LOG.printf("HEAP: post-BLE-init free=%u (baseline for leak watch)\n",
                    (unsigned)ESP.getFreeHeap());

  // BNO055 IMU — skipped in LEAN_MODE unless ENABLE_BUMP_DETECT keeps it
  // alive for accel-based impact detection. forwardWithTrim() falls back
  // to raw driveForward() via the !imuOK branch, so sequence playback
  // works without heading trim.
#if !LEAN_MODE || ENABLE_BUMP_DETECT
  bnoWire.begin(BNO_SDA, BNO_SCL);
  bnoWire.setClock(100000);
  delay(1000);
  uint8_t addrs[] = { 0x28, 0x29 };
  for (int i = 0; i < 2; i++) {
    bno = new Adafruit_BNO055(55, addrs[i], &bnoWire);
    if (bno->begin()) { bno->setMode(OPERATION_MODE_IMUPLUS); delay(200); imuOK = true; break; }
    delete bno; bno = nullptr;
  }
  SERIAL_LOG.printf("BNO055: %s\n", imuOK ? "OK" : "FAIL");
#else
  SERIAL_LOG.println("BNO055: disabled (LEAN_MODE=1, ENABLE_BUMP_DETECT=0)");
#endif

  // UWB Anchor config — skipped in LEAN_MODE (RESET held LOW above).
  // When re-enabled: T0 acts as UWB anchor ID 1 (replaces the retired
  // external A1 box). BLE role is unchanged: T0 still advertises as
  // "RobotA-Tx" with tid=0 in bit 7 of the status byte. The tag-vs-anchor
  // switch is purely a UWB PHY role: T0 stops emitting AT+RANGE lines and
  // instead passively responds to ranging pings from T1.
#if !LEAN_MODE
  sendAT("AT+RESTORE", 5000);
  sendAT("AT+SETCFG=1,1,1,1", 2000);
  sendAT("AT+SETCAP=10,10,1", 2000);
  sendAT("AT+SETRPT=1", 2000);
  // Antenna-delay calibration. 1 unit ≈ 1.5 cm of reported distance, shared
  // between both ends of the link. Stock value (16528) gives ~+100 cm bias
  // on the current MaUWB robot antennas; 16493 is a better starting point.
  // To tune: place T0 and T1 touching (antennas ~5 cm apart), read T1's raw
  // range value. Each 10 units lower reduces reported distance by ~15 cm on
  // each side → total reduction ~30 cm per 10 units. Adjust this value AND
  // the matching one in mauwb_receiver.ino (must agree on both sides).
  sendAT("AT+SETANT=16493", 2000);
  sendAT("AT+SAVE", 2000);
  sendAT("AT+RESTART", 2000);
#else
  SERIAL_LOG.println("UWB:    disabled (LEAN_MODE=1)");
#endif

  phase = PH_MANUAL;
  pushBle("BOOT");
  SERIAL_LOG.println("Starting in MANUAL mode (matrix on, motors off)");

#if AUTOPLAY
  autoplayStartAt = millis() + AUTOPLAY_DELAY_MS;
  SERIAL_LOG.printf("AUTOPLAY armed: seq=%d in %d ms\n",
                    AUTOPLAY_SEQUENCE, AUTOPLAY_DELAY_MS);
#endif
}

// ── Loop ─────────────────────────────────────────────────

void loop() {
  updateServos();
  flushScanResults();

#if LEAN_MODE
  // Belt-and-braces: re-assert DW3000 RESET low every loop. On a healthy
  // board this is a no-op. On a board with a marginal RESET joint or a
  // glitch source on GPIO 16, it clamps the line back down within ~1 ms,
  // preventing the STM32 from spuriously booting into anchor mode and
  // browning out the rail. Fix the underlying solder/trace whenever you
  // can — this is a workaround, not a cure.
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
#endif

#if AUTOPLAY
  // Pre-start countdown: flash the matrix so it's obvious the robot is
  // armed but hasn't begun moving yet. Covers the window from boot through
  // AUTOPLAY_DELAY_MS, giving you time to place the robot in its start
  // position.
  if (autoplayStartAt > 0) {
    autoplayCountdownTick();
    if (millis() >= autoplayStartAt) {
      autoplayStartAt = 0;
      matrixOff();   // clear the countdown blinker
      // Hand off to the autoplay runner. The matrix mode stays OFF until
      // the sequence's anim track runs an SOP_ANIM_* step — no more
      // auto-armed arc. If a sequence wants animation from t=0, its anim
      // track's first step should be SOP_ANIM_BOUNCE / GOL / OVOID.
      autoplayBegin(AUTOPLAY_SEQUENCE);
    }
  }
  autoplayTick();
#endif

#if !LEAN_MODE
  while (SERIAL_AT.available()) {
    char c = SERIAL_AT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (response.startsWith("AT+RANGE")) {
        SERIAL_LOG.println(response);
        parseRange(response);
      }
      response = "";
    } else {
      response += c;
    }
  }
#endif

  // ── Process BLE commands ──
  if (cmdPending) {
    cmdPending = false;
    uint8_t buf[12];
    uint8_t len = cmdLen;
    memcpy(buf, (const void*)cmdBuf, len);
    uint8_t ct = buf[0];
    int8_t p1 = len >= 2 ? (int8_t)buf[1] : 0;
    int8_t p2 = len >= 3 ? (int8_t)buf[2] : 0;

    if (ct == 0x0B) {
      driveSpeedPct = constrain(p1, 10, 100);
      SERIAL_LOG.printf("Speed: %d%%\n", driveSpeedPct);
    } else if (ct == 0x11) {
      // Motor bias / trim, applied globally to every motor command.
      // Signed int8 percent; clamped to [-50, +50]. Positive = M1+ / M2-,
      // negative = M1- / M2+. Applied symmetrically in applyMotorBias().
      motorBiasPct = (int8_t)constrain((int)p1, -50, 50);
      SERIAL_LOG.printf("Bias: %+d%% (M1 x%d%%, M2 x%d%%)\n",
                        motorBiasPct,
                        100 + motorBiasPct, 100 - motorBiasPct);
    } else if (ct == 0x0D) {
      // No-go obstacle line: 4 × int16 big-endian cm = x1,y1,x2,y2. All-zero = disabled.
      if (len >= 9) {
        noGoX1 = (int16_t)(((uint16_t)(uint8_t)buf[1] << 8) | (uint8_t)buf[2]);
        noGoY1 = (int16_t)(((uint16_t)(uint8_t)buf[3] << 8) | (uint8_t)buf[4]);
        noGoX2 = (int16_t)(((uint16_t)(uint8_t)buf[5] << 8) | (uint8_t)buf[6]);
        noGoY2 = (int16_t)(((uint16_t)(uint8_t)buf[7] << 8) | (uint8_t)buf[8]);
        lastNoGoTrig = 0;
        SERIAL_LOG.printf("NO-GO line: (%d,%d)-(%d,%d)%s\n", noGoX1, noGoY1, noGoX2, noGoY2,
          (noGoX1==0&&noGoY1==0&&noGoX2==0&&noGoY2==0) ? " DISABLED" : "");
      }
    } else if (ct == 0x0E) {
      // Path: count (0..2) + N × (xhi xlo yhi ylo) int16 BE. count=0 clears.
      if (len >= 2) {
        uint8_t n = buf[1];
        if (n > MAX_PATH_WP) n = MAX_PATH_WP;
        if (len < (uint8_t)(2 + n * 4)) n = (len - 2) / 4;
        pathLen = n;
        pathIdx = 0;
        for (int i = 0; i < n; i++) {
          int o = 2 + i * 4;
          pathWp[i].x = (int16_t)(((uint16_t)(uint8_t)buf[o]   << 8) | (uint8_t)buf[o+1]);
          pathWp[i].y = (int16_t)(((uint16_t)(uint8_t)buf[o+2] << 8) | (uint8_t)buf[o+3]);
        }
        if (pathLen == 0) {
          if (phase == PH_PATH) { motorsOff(); phase = PH_MANUAL; pushBle("PATH_CLR"); }
          SERIAL_LOG.println("PATH cleared");
        } else {
          motorsOff(); walkActive = false;
          phase = PH_PATH;
          calTracking = false;
          stallPrevTime = 0;
          bumpState = BUMP_IDLE;
          seekProbeArmed = true;   // auto-calibrate before navigating to waypoints
          seekProbing = false;
          pushBle("PATH");
          SERIAL_LOG.printf("PATH set: %d wp, first=(%d,%d)\n", pathLen, pathWp[0].x, pathWp[0].y);
        }
      }
    } else if (ct == 0x0F) {
      // SET POSE: payload = xhi xlo yhi ylo hhi hlo (3× int16 BE). Values in cm / degrees.
      // Seeds the Kalman filter to this (x,y) and sets mapBasisHdg so the robot's
      // current compass reading corresponds to it facing heading h in the map frame.
      // Use this from the map UI: click the robot's true location and drag to its
      // facing direction — far more robust than the motion probe in tight spaces.
      if (len >= 7) {
        int16_t px = (int16_t)(((uint16_t)(uint8_t)buf[1] << 8) | (uint8_t)buf[2]);
        int16_t py = (int16_t)(((uint16_t)(uint8_t)buf[3] << 8) | (uint8_t)buf[4]);
        int16_t ph = (int16_t)(((uint16_t)(uint8_t)buf[5] << 8) | (uint8_t)buf[6]);
        tagX = (float)px; tagY = (float)py;
        posValid = true;
        // Seed per-anchor Kalman filters with trig-consistent distances so we don't
        // get yanked by the next UWB sample.
        float d0 = sqrtf(tagX*tagX + tagY*tagY);
        float d1x = tagX - (float)ANCHOR_DIST_CM;
        float d1 = sqrtf(d1x*d1x + tagY*tagY);
        kfAnc[0].x = d0; kfAnc[0].P = KF_R; kfAnc[0].init = true;
        kfAnc[1].x = d1; kfAnc[1].P = KF_R; kfAnc[1].init = true;
        kfInit = true;
        lastUwbTime = millis();
        if (imuOK) {
          float c = readHeading();
          mapBasisHdg = fmodf(c - (float)ph + 360.0f, 360.0f);
        }
        calTracking = false;
        seekProbeArmed = false; seekProbing = false;  // pose is known, no need to probe
        pushBle("POSE");
        SERIAL_LOG.printf("POSE set: (%d,%d) hdg=%d° → basis=%.0f\n", px, py, ph, mapBasisHdg);
      }
    } else if (ct == 0x05) {
      motorsOff(); walkActive = false;
      phase = PH_SEEK;
      calTracking = false;
      seekPrevDistTime = 0;
      stallPrevTime = 0;
      seekOvershootUntil = 0;
      bumpState = BUMP_IDLE;
      seekWrongCount = 0;
      seekProbeArmed = true;   // auto-calibrate mapBasisHdg from motion before seeking
      seekProbing = false;
      pushBle("SEEK");
      SERIAL_LOG.println("SEEK mode (probe armed)");
    } else if (ct == 0x0A) {
      motorsOff(); walkActive = false;
      phase = PH_WANDER;
      wanderHdg = readHeading();
      wanderNextTurn = millis() + 1500 + random(2000);
      calTracking = false;
      stallPrevTime = 0;
      bumpState = BUMP_IDLE;
      pushBle("WANDER");
      SERIAL_LOG.println("WANDER mode");
    } else if (ct == 0x0C) {
      // Hard stop: kill motors AND tear down any in-flight autoplay so the
      // runner can't re-issue a step on the next loop tick. Also cancel a
      // pending startup countdown (autoplayStartAt) so Stop hit during the
      // 10-second arm window aborts boot-time playback. Matrix is parked
      // off so the OFF state is visually unambiguous.
      motorsOff(); walkActive = false;
      phase = PH_MANUAL;
      autoplayActive  = false;
      autoplayStartAt = 0;
      autoMotor       = nullptr;
      autoAnim        = nullptr;
      autoMotorIdx    = 0;
      autoAnimIdx     = 0;
      matrixSetMode(MMODE_OFF, 0, 0);
      pushBle("STOP");
      SERIAL_LOG.println("STOP");
    } else if (ct == 0x01 || ct == 0x02 || ct == 0x08) {
      // Direct motor-control commands only — switch to MANUAL here.
      if (phase != PH_MANUAL) {
        phase = PH_MANUAL; motorsOff();
        pushBle("MANUAL_ON");
      }
      if (ct == 0x01) {
        walkActive = false;
        motor1_us = speedToUs(p1, M1_FWD);
        motor2_us = speedToUs(p2, M2_FWD);
      } else if (ct == 0x02) {
        walkActive = true; walkM1Turn = true; walkStepStart = millis();
      } else /* ct == 0x08 */ {
        walkActive = false;
        turnInPlace(p1);
      }
    } else if (ct == 0x04 && dfPlayerOK) {
      dfPlayer.play(constrain(p1, 1, 32));
    } else if (ct == 0x06) {
      mapBasisHdg = readHeading(); targetHdg = mapBasisHdg;
      SERIAL_LOG.printf("Calibrate: mapBasis=%.1f\n", mapBasisHdg);
    } else if (ct == 0x07) {
      // p1 packs (mode << 4) | cmplx. Modes:
      //   0 = off, 1 = bouncers, 2 = GoL, 3 = ovoid-merge.
      // p2 = anim speed (5-50, 10 ms units).
      // Mirrors the SOP_ANIM_* autoplay ops; same matrixSetMode() path.
      uint8_t mode  = (p1 >> 4) & 0x0F;
      uint8_t cmplx =  p1       & 0x0F;
      const char* modeName = "?";
      switch (mode) {
        case 0: matrixSetMode(MMODE_OFF,          0,     p2); modeName = "off";     break;
        case 1: matrixSetMode(MMODE_BOUNCE,       cmplx, p2); modeName = "bounce";  break;
        case 2: matrixSetMode(MMODE_GOL,          cmplx, p2); modeName = "gol";     break;
        case 3: matrixSetMode(MMODE_OVOID_MERGE,  0,     p2); modeName = "ovoid";   break;
        default: break;
      }
      SERIAL_LOG.printf("Matrix: mode=%s cmplx=%u speed=%lums/frame\n",
                        modeName, (unsigned)cmplx, matrixSpeedMs);
    } else if (ct == 0x09) {
      // Matrix off + walk stop (dual-purpose)
      matrixSetMode(MMODE_OFF, 0, 0);
      walkActive = false; motorsOff();
    }
    // Unknown command: ignore silently (don't force PH_MANUAL).
  }

  // ── MANUAL: remote control ──
  if (phase == PH_MANUAL) {
    // Collision handling during browser-driven sequence playback: if a bump
    // is already being recovered, let it finish (overrides motor commands);
    // otherwise sniff for new impacts. On bump, recovery backs up + turns,
    // then stops motors — the next sequence step from the browser re-drives.
    if (bumpState != BUMP_IDLE) {
      handleBumpRecovery();
    } else {
      checkAccelBump(PH_MANUAL);
      if (walkActive) {
        if (millis() - walkStepStart >= WALK_STEP_MS) {
          walkM1Turn = !walkM1Turn;
          walkStepStart = millis();
        }
        motor1_us = walkM1Turn ? scaledM1() : 0;
        motor2_us = walkM1Turn ? 0 : scaledM2();
      }
    }
    matrixAnimTick();
    checkBleEvents();
    return;
  }

  // ── WAIT: until first UWB reading ──
  if (phase == PH_WAIT_UWB) {
    if (kfInit) {
      targetHdg     = readHeading();
      prevCheckDist = kf_x;
      prevCheckTime = millis();
      nudgeCount    = 0;
      nudgeDir      = 1;
      phase = PH_DRIVE;
      driveStartTime = millis();
      pushBle("GO");
      SERIAL_LOG.printf("GO  hdg=%.1f  dist=%.0fcm\n", targetHdg, kf_x);
    }
    if (millis() - lastLog >= 1000) {
      lastLog = millis();
      SERIAL_LOG.printf("WAIT  imu=%s  uwb=%s\n", imuOK ? "OK" : "--", kfInit ? "OK" : "--");
    }
    checkBleEvents();
    return;
  }

  // ── DRIVE: half speed + heading trim + distance gradient ──
  if (phase == PH_DRIVE) {
    if (millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      phase = PH_WAIT_UWB;
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
      SERIAL_LOG.println("UWB lost");
      return;
    }

    if (kfInit && kf_x < ARRIVE_DIST && millis() - driveStartTime > MIN_DRIVE_MS) {
      motorsOff();
      phase = PH_ARRIVED;
      pushBle("ARRIVED");
      SERIAL_LOG.printf("ARRIVED  dist=%.0fcm\n", kf_x);
      return;
    }

    // Distance-gradient course correction
    if (kfInit && millis() - prevCheckTime >= DIST_CHECK_MS) {
      if (kf_x > prevCheckDist + FARTHER_THRESH) {
        targetHdg += nudgeDir * NUDGE_DEG;
        if (targetHdg >= 360) targetHdg -= 360;
        if (targetHdg < 0)    targetHdg += 360;
        nudgeCount++;
        if (nudgeCount >= MAX_NUDGE) {
          nudgeDir = -nudgeDir;
          nudgeCount = 0;
        }
        SERIAL_LOG.printf("NUDGE hdg→%.1f dist=%.0f\n", targetHdg, kf_x);
      } else if (kf_x < prevCheckDist - FARTHER_THRESH) {
        nudgeCount = 0;
      }
      prevCheckDist = kf_x;
      prevCheckTime = millis();
    }

    float drift = imuOK ? headingDiff(readHeading(), targetHdg) : 0;
    if (imuOK) forwardWithTrim(drift); else driveForward();

    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("DRIVE  dist=%.0f  drift=%+.1f  m1=%d m2=%d\n",
        kf_x, drift, motor1_us, motor2_us);
    }
    checkBleEvents();
    return;
  }

  // ── SEEK: navigate toward the other robot ──
  if (phase == PH_SEEK) {
    if (handleBumpRecovery()) { checkBleEvents(); return; }
    checkAccelBump(PH_SEEK);
    checkNoGo(PH_SEEK);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); checkBleEvents(); return; }

    // UWB timeout → reset and keep seeking
    if (kfInit && millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
      SERIAL_LOG.println("SEEK: UWB lost, waiting for fix...");
    }

    // Need own position first (peer not required for the probe)
    if (!posValid) {
      motorsOff();
      if (millis() - lastLog >= 1000) {
        lastLog = millis();
        SERIAL_LOG.println("SEEK wait: no own position");
      }
      checkBleEvents();
      return;
    }

    // ── Motion-probe auto-calibration ──────────────────────
    // Drive straight for ~25 cm and derive mapBasisHdg empirically from the
    // direction of travel. Far more accurate than user eyeballing alignment.
    if (seekProbeArmed) {
      if (!seekProbing) {
        seekProbeX0 = tagX;
        seekProbeY0 = tagY;
        seekProbeC0 = imuOK ? readHeading() : 0;
        seekProbeStartT = millis();
        seekProbing = true;
        SERIAL_LOG.printf("PROBE start: pos=(%.0f,%.0f) comp=%.0f\n",
                          seekProbeX0, seekProbeY0, seekProbeC0);
      }
      driveForward();  // straight, no IMU trim (trim would bias the motion vector)

      float mdx = tagX - seekProbeX0, mdy = tagY - seekProbeY0;
      float moved = sqrtf(mdx * mdx + mdy * mdy);
      bool timeout = millis() - seekProbeStartT > PROBE_TIMEOUT_MS;
      bool enough  = moved >= PROBE_TARGET_MOVE_CM;

      if (enough || timeout) {
        if (moved >= PROBE_MIN_MOVE_CM && imuOK) {
          float alpha = atan2f(mdy, mdx) * 180.0f / M_PI;
          mapBasisHdg = fmodf(seekProbeC0 - alpha + 360.0f, 360.0f);
          seekWrongCount = 0;
          calTracking = false;
          SERIAL_LOG.printf("PROBE OK: moved=%.0fcm α=%+.0f → basis=%.0f\n",
                            moved, alpha, mapBasisHdg);
        } else {
          SERIAL_LOG.printf("PROBE fail: only %.0fcm in %lums — wrong-way will recover\n",
                            moved, millis() - seekProbeStartT);
        }
        seekProbeArmed = false;
        seekProbing = false;
        motorsOff();
      }
      checkStall(PH_SEEK);   // still detect bumps during probe
      checkBleEvents();
      return;
    }

    // Now need peer position too
    bool peerFresh = otherPosValid && (millis() - lastOtherUpdate < OTHER_TIMEOUT_MS);
    if (!peerFresh) {
      motorsOff();
      if (millis() - lastLog >= 1000) {
        lastLog = millis();
        SERIAL_LOG.println("SEEK wait: no peer");
      }
      checkBleEvents();
      return;
    }

    float dx = otherX - tagX;
    float dy = otherY - tagY;
    float tagDist = sqrtf(dx * dx + dy * dy);

    // UWB reports arrival within its ~10–30 cm accuracy floor; commit the
    // final gap with a speed-scaled forward overshoot so the robots actually touch.
    if (seekOvershootUntil == 0 && tagDist < SEEK_ARRIVE_DIST) {
      unsigned long ms = 33333UL / (unsigned long)driveSpeedPct;  // ≈10 cm @ ~30 cm/s full speed
      if (ms < 200)  ms = 200;
      if (ms > 3000) ms = 3000;
      seekOvershootUntil = millis() + ms;
      SERIAL_LOG.printf("SEEK arrive d=%.0fcm, overshoot %lums\n", tagDist, ms);
    }

    if (seekOvershootUntil) {
      if (millis() >= seekOvershootUntil) {
        motorsOff();
        phase = PH_ARRIVED;
        seekOvershootUntil = 0;
        pushBle("ARRIVED");
        SERIAL_LOG.printf("MET! final d=%.0fcm\n", tagDist);
        checkBleEvents();
        return;
      }
      float drift = imuOK ? headingDiff(readHeading(), targetHdg) : 0;
      if (imuOK) forwardWithTrim(drift); else driveForward();
      checkBleEvents();
      return;
    }

    // Desired heading in map coords → compass heading.
    // BNO055 is CCW-positive (compass increases with CCW rotation, same direction
    // as atan2 angles), so: targetCompass = mapBasisHdg + α.
    float mapAngleDeg = atan2f(dy, dx) * 180.0f / M_PI;
    targetHdg = fmodf(mapBasisHdg + mapAngleDeg + 360.0f, 360.0f);

    // drift = target − current (CCW-positive): drift>0 means target is CCW of us.
    float drift = imuOK ? headingDiff(readHeading(), targetHdg) : 0;

    // Distance-gradient correction: if we're getting farther while driving forward,
    // mapBasisHdg is wrong. First error: flip 180° (most common case — peer direction
    // sign inverted). Subsequent errors: step 90° until distance starts to shrink.
    if (seekPrevDistTime == 0) { seekPrevDist = tagDist; seekPrevDistTime = millis(); }
    if (fabs(drift) < 25 && millis() - seekPrevDistTime >= 1000) {  // was 1500
      float delta = tagDist - seekPrevDist;
      if (delta > 3.0f) {                                          // was 4
        float nudge = (seekWrongCount == 0) ? 180.0f : 90.0f;
        mapBasisHdg = fmodf(mapBasisHdg + nudge + 360.0f, 360.0f);
        seekWrongCount++;
        if (seekWrongCount > 4) seekWrongCount = 0;   // full circle, start over
        calTracking = false;                          // force fresh auto-cal baseline
        SERIAL_LOG.printf("SEEK wrong-way (#%d): Δ=%+.0f → basis +%.0f = %.0f\n",
                          seekWrongCount, delta, nudge, mapBasisHdg);
      } else if (delta < -2.0f) {
        seekWrongCount = 0;  // getting closer — lock in this basis
      }
      seekPrevDist = tagDist; seekPrevDistTime = millis();
    }

    if (fabs(drift) > 30) { spinToward(drift); stallPrevTime = 0; }
    else {
      if (imuOK) forwardWithTrim(drift); else driveForward();
      maybeAutoCalibrate(drift);
      checkStall(PH_SEEK);
    }

    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("SEEK d=%.0f basis=%.0f hdg=%.0f drift=%+.1f me=(%.0f,%.0f) peer=(%.0f,%.0f)\n",
        tagDist, mapBasisHdg, targetHdg, drift, tagX, tagY, otherX, otherY);
    }
    checkBleEvents();
    return;
  }

  // ── PATH: follow user-drawn waypoints ──────────────────
  if (phase == PH_PATH) {
    if (handleBumpRecovery()) { checkBleEvents(); return; }
    checkAccelBump(PH_PATH);
    checkNoGo(PH_PATH);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); checkBleEvents(); return; }

    if (kfInit && millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
    }
    if (!posValid) {
      motorsOff(); checkBleEvents(); return;
    }

    // Motion-probe auto-calibration (same as SEEK). Runs only on first entry.
    if (seekProbeArmed) {
      if (!seekProbing) {
        seekProbeX0 = tagX; seekProbeY0 = tagY;
        seekProbeC0 = imuOK ? readHeading() : 0;
        seekProbeStartT = millis();
        seekProbing = true;
        SERIAL_LOG.printf("PATH PROBE start: pos=(%.0f,%.0f) comp=%.0f\n",
                          seekProbeX0, seekProbeY0, seekProbeC0);
      }
      driveForward();
      float mdx = tagX - seekProbeX0, mdy = tagY - seekProbeY0;
      float moved = sqrtf(mdx * mdx + mdy * mdy);
      if (moved >= PROBE_TARGET_MOVE_CM || millis() - seekProbeStartT > PROBE_TIMEOUT_MS) {
        if (moved >= PROBE_MIN_MOVE_CM && imuOK) {
          float alpha = atan2f(mdy, mdx) * 180.0f / M_PI;
          mapBasisHdg = fmodf(seekProbeC0 - alpha + 360.0f, 360.0f);
          calTracking = false;
          SERIAL_LOG.printf("PATH PROBE OK: moved=%.0f α=%+.0f → basis=%.0f\n",
                            moved, alpha, mapBasisHdg);
        } else {
          SERIAL_LOG.printf("PATH PROBE fail: only %.0fcm\n", moved);
        }
        seekProbeArmed = false; seekProbing = false;
        motorsOff();
      }
      checkStall(PH_PATH);
      checkBleEvents();
      return;
    }

    if (pathIdx >= pathLen) {
      motorsOff();
      phase = PH_ARRIVED;
      pushBle("PATH_DONE");
      SERIAL_LOG.println("PATH done");
      checkBleEvents();
      return;
    }

    float dx = (float)pathWp[pathIdx].x - tagX;
    float dy = (float)pathWp[pathIdx].y - tagY;
    float wpDist = sqrtf(dx * dx + dy * dy);
    if (wpDist < WP_ARRIVE_DIST) {
      SERIAL_LOG.printf("PATH wp %d reached (d=%.0f), next\n", pathIdx, wpDist);
      pathIdx++;
      motorsOff();
      calTracking = false;
      return;
    }

    float mapAngleDeg = atan2f(dy, dx) * 180.0f / M_PI;
    targetHdg = fmodf(mapBasisHdg + mapAngleDeg + 360.0f, 360.0f);
    float drift = imuOK ? headingDiff(readHeading(), targetHdg) : 0;

    if (fabs(drift) > 30) { spinToward(drift); stallPrevTime = 0; }
    else {
      if (imuOK) forwardWithTrim(drift); else driveForward();
      maybeAutoCalibrate(drift);
      checkStall(PH_PATH);
    }

    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("PATH wp%d/%d d=%.0f hdg=%.0f drift=%+.1f\n",
        pathIdx, pathLen, wpDist, targetHdg, drift);
    }
    checkBleEvents();
    return;
  }

  // ── WANDER: random exploration with collision avoidance ──
  if (phase == PH_WANDER) {
    if (handleBumpRecovery()) { matrixAnimTick(); checkBleEvents(); return; }
    checkAccelBump(PH_WANDER);
    checkNoGo(PH_WANDER);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); matrixAnimTick(); checkBleEvents(); return; }

    if (kfInit && millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
    }

    if (millis() >= wanderNextTurn) {
      wanderHdg += -45 + random(91);
      if (wanderHdg >= 360) wanderHdg -= 360;
      if (wanderHdg < 0) wanderHdg += 360;
      wanderNextTurn = millis() + 2000 + random(3000);
    }

    targetHdg = wanderHdg;
    float drift = imuOK ? headingDiff(readHeading(), targetHdg) : 0;
    if (fabs(drift) > 30) { spinToward(drift); stallPrevTime = 0; }
    else {
      if (imuOK) forwardWithTrim(drift); else driveForward();
      maybeAutoCalibrate(drift);
      checkStall(PH_WANDER);
    }

    matrixAnimTick();
    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("WANDER hdg=%.0f drift=%+.1f pos=(%.0f,%.0f) basis=%.0f\n",
        wanderHdg, drift, tagX, tagY, mapBasisHdg);
    }
    checkBleEvents();
    return;
  }

  // ── ARRIVED: stay stopped ──
  if (phase == PH_ARRIVED) {
    motorsOff();
    if (millis() - lastLog >= 2000) {
      lastLog = millis();
      SERIAL_LOG.printf("ARRIVED  dist=%.0f\n", kf_x);
    }
    checkBleEvents();
  }
}
