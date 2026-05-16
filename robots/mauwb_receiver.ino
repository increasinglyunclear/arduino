/*
 * Robot-B (Receiver) — v4: THE ONE WITH THE STONE.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <DFRobotDFPlayerMini.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// DEBUG BUILD: serial logging enabled. For T1 the most useful output is
// the raw AT+RANGE=tid:1,mask:NN,...,range:(...) lines — they show which
// anchors responded this cycle and the per-anchor distance in cm. To
// silence later for untethered runs:
//   #define SERIAL_LOG if (0) Serial
#define SERIAL_LOG Serial
HardwareSerial SERIAL_AT(2);

#define RESET_PIN   16
#define IO_RXD2     18
#define IO_TXD2     17
#define I2C_SDA     39
#define I2C_SCL     38
#define BNO_SDA     5
#define BNO_SCL     6
#define MOTOR1_PIN  7
#define MOTOR2_PIN  8
#define STATUS_LED  9
#define IR_LED_PIN  42
#define DFPLAYER_TX 1
#define DFPLAYER_RX 2

// ── Peripheral enable flags ──────────────────────────────
// Set to 0 to skip initialisation and all runtime calls. Used to shed
// current / boot-time spikes on a brownout-prone supply. Flip back to 1
// if you wire a real speaker / re-enable the IR illuminator.
//
// ENABLE_DFPLAYER:
//   Skipping Serial1 + dfPlayer.begin() avoids the ~500 ms handshake
//   during which the DFPlayer spins up its SD card and biases its audio
//   amp — a significant transient on a shared regulator. Truly cutting
//   DFPlayer current draw requires a MOSFET on its VCC rail; this flag
//   is the software half of the fix.
// ENABLE_IR_LED:
//   The IR illuminator is held HIGH continuously (~20 mA steady state)
//   for the camera-based vision experiments. Disable while debugging
//   pure UWB seek to reclaim that margin.
#define ENABLE_DFPLAYER 0
#define ENABLE_IR_LED   1

// LEAN_MODE:
//   Suspend UWB ranging, BNO055 IMU, and the peer-scan trilateration path
//   on the robots. Only the BLE command channel (AnchorCmd) and motor /
//   LED output stay live. Cuts the biggest battery load (the DW3000
//   STM32's always-on receiver in anchor mode, and the IMU's I²C traffic)
//   so the robots can focus on record + playback of motor + LED sequences
//   without browning out. BLE scan stays on — it's how commands arrive.
//   Flip to 0 to restore full UWB / seek / trilateration behaviour.
#define LEAN_MODE 1

// ENABLE_BUMP_DETECT:
//   Keep the BNO055 IMU alive even under LEAN_MODE so accelerometer-based
//   impact detection can fire during browser-driven sequence playback.
//   BNO055 in IMU mode draws ~12 mA — small compared to the ~100 mA UWB
//   load that LEAN_MODE eliminates. Set to 0 to drop another ~12 mA if
//   the supply still browns out and you're OK running blind into walls.
#define ENABLE_BUMP_DETECT 0

// ── Autoplay: hard-coded sequence on boot ───────────────
// See mauwb_emitter.ino for the full rationale. Keep this block and
// AUTOPLAY_TABLE in this file in sync with the emitter's copy.
//   1 = Finding          5 = I See You
//   2 = Missed Connections  6 = Circling
//   3 = Levy A           7 = Chemotaxis
//   4 = Levy B
#define AUTOPLAY             1
#define AUTOPLAY_DELAY_MS    10000
#define AUTOPLAY_SEQUENCE    9
#define AUTOPLAY_REPEAT      1   // 1 = once, 2+ = repeat N times, 0 = loop forever

// ── OLED matrix animation (T1 only) ──────────────────────
// Mirrors T0's MAX7219 8x8 engine onto the 128x64 SSD1306 via an 8x scaled
// 64x64 render. The OLED render is disabled by default because each frame
// ships a 1 KB I2C framebuffer (~80 ms blocking at 100 kHz) which collides
// with the 50 Hz servo update loop and causes visible motor jitter. Set to
// 1 and reflash to re-enable. The matrix ENGINE keeps running regardless
// (cheap, ~300 µs/frame) so ledMode==4 ("GoL pixel") still has live
// dispBuf data to tap when the OLED is off.
#define T1_OLED_RENDER       0

// ── Motor tuning ─────────────────────────────────────────
const int MOTOR_STOP = 1500;
const int M1_FWD  = 1000;
const int M2_FWD  = 2000;
const int BIAS = 0;

const float TRIM_GAIN    = 6.0;
const int   TRIM_MAX     = 300;
const float DEADBAND_DEG = 2.0;

int motor1_us = 0, motor2_us = 0;
portMUX_TYPE pwmMux = portMUX_INITIALIZER_UNLOCKED;

// Motor bias / trim (set at runtime via command 0x11).
// Range [-50, +50] percent. Applied in updateServos() so every motor
// command path — manual drive, walk, turn, seek, wander, bump recovery,
// sequence playback — inherits the same correction. Weakens one wheel
// relative to the other; never boosts past the calibrated endpoints.
//   bias > 0  → cut M2 by bias%
//   bias < 0  → cut M1 by |bias|%
int8_t motorBiasPct = 0;

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

// ── State ────────────────────────────────────────────────
enum Phase { PH_WAIT_UWB, PH_DRIVE, PH_ARRIVED, PH_MANUAL, PH_SEEK, PH_WANDER, PH_PATH };
Phase phase = PH_WAIT_UWB;
float targetHdg = 0;
const float ARRIVE_DIST = 10.0;
unsigned long driveStartTime = 0;
const unsigned long MIN_DRIVE_MS = 3000;

// ── Path following ──────────────────────────────────────
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

#define MY_ROBOT_ID 1
volatile uint8_t lastAnchorSeq = 0;
uint8_t driveSpeedPct = 70;

float wanderHdg = 0;
unsigned long wanderNextTurn = 0;

// Seek arrival + overshoot
const float SEEK_ARRIVE_DIST = 25.0f;
const int   OVERSHOOT_CM = 10;
unsigned long seekOvershootUntil = 0;

// Auto-calibration
float calPrevX = 0, calPrevY = 0;
unsigned long calPrevTime = 0;
bool calTracking = false;

// Seek distance-gradient correction
float seekPrevDist = 0;
unsigned long seekPrevDistTime = 0;

// ── Seek motion-probe auto-calibration ──────────────────
// On SEEK entry drive straight ~25 cm and derive mapBasisHdg from observed
// UWB motion (C0 − α_obs). More accurate than manual chassis alignment.
bool seekProbeArmed = false;
bool seekProbing = false;
unsigned long seekProbeStartT = 0;
float seekProbeX0 = 0, seekProbeY0 = 0;
float seekProbeC0 = 0;
const float PROBE_MIN_MOVE_CM    = 15.0f;
const float PROBE_TARGET_MOVE_CM = 25.0f;
const unsigned long PROBE_TIMEOUT_MS = 4000;

// ── Bump / stall recovery ───────────────────────────────
enum BumpState { BUMP_IDLE, BUMP_BACKING, BUMP_TURNING };
BumpState bumpState = BUMP_IDLE;
unsigned long bumpEndTime = 0;
unsigned long bumpBackMs = 800;          // current recovery's back-up duration
unsigned long bumpTurnTimeoutMs = 2200;  // current recovery's turn timeout
float bumpPrevHdg = 0, bumpCumTurn = 0;
float bumpTargetAngle = 120;             // current recovery's target turn angle (deg)
int bumpDir = 1;
Phase bumpReturnPhase = PH_MANUAL;
// Escalation: if a new bump fires within BUMP_REPEAT_WINDOW ms of the last
// one finishing, we're probably wedged — back up longer, turn the OTHER
// way, and after a few repeats bail out of the seek entirely.
unsigned long bumpLastDoneAt = 0;
uint8_t bumpRepeatCount = 0;
const unsigned long BUMP_REPEAT_WINDOW_MS = 2000;
const uint8_t BUMP_REPEAT_GIVEUP = 4;

float stallPrevX = 0, stallPrevY = 0;
unsigned long stallPrevTime = 0;
// Stall detection is the *only* bump detector (accelerometer disabled). Since
// 2D trilateration no longer runs (T0 is a mobile anchor), we detect stall
// via changes in the *measured* UWB distances to T0 and A0. If T1 is
// commanding forward motion but neither distance is changing, T1 is jammed
// against something. Either distance changing counts as real motion, so the
// check is robust to the robot driving tangent to one of the two beacons.
//
// Used by PH_WANDER/PH_PATH (continuous straight driving). During PH_SEEK,
// stall is detected inline at the end of each SS_DRIVE segment using the
// same threshold — see seekStallStreak below. Running this helper in
// parallel with the inline check caused false triggers when UWB geometry
// made both radial distances momentarily stable.
float stallPrevDT0 = 0;  // last sampled d(T0,T1)
float stallPrevDA0 = 0;  // last sampled d(T1,A0); 0 if A0 not seen
const float STALL_MIN_MOVE_CM = 5.0;
const unsigned long STALL_WINDOW_MS = 900;

// ── Accelerometer-based impact detection ────────────────
unsigned long drivingSince = 0;
unsigned long lastAccelCheck = 0;
const float ACCEL_IMPACT_MPS2 = 1.5f;       // more sensitive (was 2.0)
const unsigned long DRIVE_BLANK_MS = 400;   // shorter blanking (was 500)
const unsigned long ACCEL_CHECK_MS = 25;    // ~40 Hz (was ~33)

// Direction-aware: motors are mirrored (M1_FWD and M2_FWD are on opposite sides
// of MOTOR_STOP), so a simple "both > MOTOR_STOP+50" check is always false.
// This broke checkAccelBump AND checkStall — collision detection NEVER fired.
inline bool isDrivingForward() {
  int m1Delta = (M1_FWD > MOTOR_STOP) ? (motor1_us - MOTOR_STOP) : (MOTOR_STOP - motor1_us);
  int m2Delta = (M2_FWD > MOTOR_STOP) ? (motor2_us - MOTOR_STOP) : (MOTOR_STOP - motor2_us);
  return motor1_us > 0 && motor2_us > 0 && m1Delta > 50 && m2Delta > 50;
}

// ── No-go obstacle (line segment drawn on map) ──────────
volatile int16_t noGoX1 = 0, noGoY1 = 0, noGoX2 = 0, noGoY2 = 0;
unsigned long lastNoGoTrig = 0;
const unsigned long NOGO_COOLDOWN_MS = 4000;
const float NOGO_THRESH_CM = 25.0f;

// ── Distance calibration ───────────────────────────────
// Offset (cm) subtracted from the raw UWB d(T0,T1) before any seek logic
// runs, so the firmware's arrival threshold reflects the user's calibrated
// zero rather than raw ranging (which has ~10–20 cm antenna-delay bias
// per hardware set). The browser sends this via command 0x10 whenever the
// user hits Calibrate / Reset, and on page-load to push the persisted
// localStorage value back to T1 after a reflash. Raw d is still broadcast
// over BLE so the browser can do its own display-side calibration.
volatile int16_t distOffsetCm = 0;

// ── Distance-gradient seek state machine ───────────────────
// With T0 as a mobile UWB anchor we have only one usable signal: d(T0,T1).
// Algorithm (rate-comparison hill climb):
//   SS_DRIVE  — drive forward SEEK_DRIVE_MS, sample d(T0,T1) at start/end.
//               Compare Δd with the *previous* segment's Δd. Each pair
//               of (turn → drive) tells us whether the turn improved the
//               closing rate or hurt it:
//                 - Closing fast (Δd < −SEEK_PROGRESS_CM): keep going.
//                 - Turn helped (Δd more negative than before): keep
//                   turning the same direction next time — we're climbing.
//                 - Turn hurt  (Δd more positive than before): flip turn
//                   direction — we just passed the optimum.
//                 - Indeterminate: keep committed direction (explore).
//   SS_TURN   — rotate in place for SEEK_TURN_MS. After turning, re-probe
//               in SS_DRIVE. The rate comparison above decides the next
//               turn direction.
//   Stall     — if |Δd_T0| and |Δd_A0| are both under STALL_MIN_MOVE_CM at
//               the end of a drive segment, we're physically stuck; fire
//               bump recovery. This is an explicit check that complements
//               the time-window checkStall() helper.
//   Arrival   — d < SEEK_ARRIVE_DIST → timed blind overshoot (UWB floor).
enum SeekStep { SS_DRIVE, SS_TURN };
SeekStep seekStep = SS_DRIVE;
unsigned long seekStepStart = 0;  // 0 = state hasn't latched its start yet
float seekStartDist = 0;          // d(T0,T1) at the beginning of current drive segment
float seekStartDistA0 = 0;        // d(T1,A0) at the beginning — for stall detection
uint8_t seekWrongCount = 0;       // # consecutive non-improving segments
int8_t  seekTurnDir = +1;         // committed turn direction (hill-climb state)
// Rate-comparison memory
float   seekPrevDelta = 0;        // Δd of the previous drive segment (cm, negative = closing)
bool    seekHasPrevDelta = false;
bool    seekJustTurned = false;   // true if this drive follows a SS_TURN
// Stall requires TWO consecutive drive segments with essentially no motion.
// One-off coincidences (e.g., T1 happens to move tangent to both A0 and T0
// in a single segment, or a UWB noise spike) don't count.
uint8_t seekStallStreak = 0;
const unsigned long SEEK_DRIVE_MS  = 600;  // forward probe length
const unsigned long SEEK_TURN_MS   = 200;  // fixed turn chunk; ~30° at 70% on stock motors
const float         SEEK_PROGRESS_CM = 3.0f;  // min distance drop to count as progress
const float         SEEK_RATE_HYST_CM = 1.5f; // Δd must differ by this to count as better/worse
// After this many non-improving segments in a row we assume a full 360° has
// been swept; reverse the turn direction once as a fallback.
const uint8_t       SEEK_SWEEP_FULL = 12;

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

#if AUTOPLAY
// Peer (T0) sequence-sync state. Written by PeerScanCB::onResult() each
// time we receive a fresh "RobotA-Tx" advertisement; read by
// syncToPeer() from the main loop. Declared above PeerScanCB so the
// class body's references resolve at parse time. uint8_t / uint16_t
// reads on ESP32 are atomic for naturally-aligned values, so volatile
// is enough; no mutex needed.
volatile bool          peerSyncValid    = false;
volatile uint8_t       peerMotorIdx     = 0xFF;
volatile uint8_t       peerAnimIdx      = 0xFF;
volatile uint16_t      peerMotorMsLeft  = 0;
volatile unsigned long peerSyncRxMs     = 0;
#endif

class PeerScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveName()) return;
    String name = dev.getName().c_str();

    // T0 broadcast: parses two things off the same advertisement:
    //   1) (non-LEAN_MODE) UWB d0/d1 for trilateration of T0's position.
    //   2) Sequence-sync bytes 8..11 → consumed in syncToPeer().
    // mf payload layout (after 0xFF 0xFF prefix):
    //   [0..1] d0  [2..3] d1  [4] status  [5] hdg
    //   [6] motorIdx  [7] animIdx  [8..9] motorMsLeft (BE)
    if (name == "RobotA-Tx") {
      if (!dev.haveManufacturerData()) return;
      String mfRaw = dev.getManufacturerData();
      const uint8_t* mf = (const uint8_t*)mfRaw.c_str();
      int len = mfRaw.length();
      int off = (len >= 8 && mf[0] == 0xFF && mf[1] == 0xFF) ? 2 : 0;
      if (len - off < 6) return;

#if !LEAN_MODE
      uint16_t pd0 = ((uint16_t)mf[off + 0] << 8) | mf[off + 1];
      uint16_t pd1 = ((uint16_t)mf[off + 2] << 8) | mf[off + 3];
      if (pd0 != 0 && pd1 != 0) {
        float D = ANCHOR_DIST_CM;
        float fx = ((float)pd0 * pd0 - (float)pd1 * pd1 + D * D) / (2.0f * D);
        float fy2 = (float)pd0 * pd0 - fx * fx;
        if (fy2 >= 0) { otherX = fx; otherY = sqrtf(fy2); otherPosValid = true; lastOtherUpdate = millis(); }
      }
#endif

#if AUTOPLAY
      // Sync bytes are present iff the emitter's payload was extended
      // (>= 12 bytes total → >= 10 after the 0xFF prefix). Older firmware
      // sending the legacy 8-byte payload simply doesn't update sync
      // state and we keep running standalone. Gated on AUTOPLAY so the
      // peer state vars (defined alongside the runner) exist.
      if (len - off >= 10) {
        peerMotorIdx    = mf[off + 6];
        peerAnimIdx     = mf[off + 7];
        peerMotorMsLeft = ((uint16_t)mf[off + 8] << 8) | mf[off + 9];
        peerSyncRxMs    = millis();
        peerSyncValid   = true;
      }
#endif
      return;
    }

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
unsigned long arrivedTime = 0;
bool audioPlayed = false;

// ── Helpers ──────────────────────────────────────────────

String sendAT(const String& cmd, unsigned long timeout) {
  SERIAL_LOG.print(">> "); SERIAL_LOG.println(cmd);
  SERIAL_AT.println(cmd);
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (SERIAL_AT.available()) resp += (char)SERIAL_AT.read();
  }
  // Echo the STM32's reply so boot is debuggable over USB.
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
    if (ancids[i] >= 0 && ancids[i] <= 1 && ranges[i] > 0) {
      kfAnc[ancids[i]].feed((float)ranges[i]);
    }
  }
  trilaterateTag();
}

// ── Motor helpers ────────────────────────────────────────

// Soft-start slew on motor pulse widths — eliminates STOP→FULL current transient
// that was triggering brownouts. STOP→FULL now takes ~120 ms (imperceptible).
int motor1_us_out = 0, motor2_us_out = 0;
const int MOTOR_SLEW_US = 80;

static inline int slewToward(int cur, int tgt) {
  if (cur == tgt) return tgt;
  if (cur == 0 && tgt > 0)      return MOTOR_STOP;
  if (cur > 0  && tgt == 0)     return 0;
  if (cur < tgt) return min(cur + MOTOR_SLEW_US, tgt);
  else           return max(cur - MOTOR_SLEW_US, tgt);
}

// Apply motorBiasPct to a single motor's target pulse.
// `us`==0 (off) and `us`==MOTOR_STOP (neutral) are preserved. Positive bias
// boosts M1 by bias% and cuts M2 by bias% (symmetric split — see
// mauwb_emitter.ino for the full rationale). Final pulse-width delta is
// clamped to the calibrated endpoint magnitude so a boost can't over-drive
// the servo past its forward/reverse limit.
static inline int applyMotorBias(int us, bool isM1) {
  if (us == 0 || us == MOTOR_STOP || motorBiasPct == 0) return us;
  int mult = 100 + (isM1 ? motorBiasPct : -motorBiasPct);
  if (mult < 0) mult = 0;
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
    motor1_us = m1;
    motor2_us = constrain(m2 - trim, MOTOR_STOP, m2);
  } else if (drift < -DEADBAND_DEG) {
    motor1_us = constrain(m1 + trim, m1, MOTOR_STOP);
    motor2_us = m2;
  } else {
    motor1_us = m1; motor2_us = m2;
  }
}

// Spin in place: M1=right wheel, M2=left wheel for Receiver
void spinToward(float drift) {
  if (drift > 0) { motor1_us = scaledM1(); motor2_us = 0; }
  else            { motor1_us = 0; motor2_us = scaledM2(); }
}

void driveBackward() {
  motor1_us = MOTOR_STOP + (MOTOR_STOP - M1_FWD) * driveSpeedPct / 100;
  motor2_us = MOTOR_STOP + (MOTOR_STOP - M2_FWD) * driveSpeedPct / 100;
}

// Signed turn-in-place: +rate = CCW (left), -rate = CW (right), magnitude 0-100.
// Receiver: M1=right wheel, M2=left wheel. Counter-rotation: both wheels
// driven in opposite directions for a true spin-in-place (was pivot: one
// wheel off, one driven). Halves the turn radius and doubles the angular
// velocity at the same `rate` value.
void turnInPlace(int8_t rate) {
  if (rate == 0) { motorsOff(); return; }
  int mag = rate > 0 ? rate : -rate;
  if (rate > 0) {
    // CCW: M1 forward (right wheel fwd), M2 reverse (left wheel rev)
    motor1_us = MOTOR_STOP + (M1_FWD - MOTOR_STOP) * mag / 100;
    motor2_us = MOTOR_STOP - (M2_FWD - MOTOR_STOP) * mag / 100;
  } else {
    // CW: M1 reverse, M2 forward
    motor1_us = MOTOR_STOP - (M1_FWD - MOTOR_STOP) * mag / 100;
    motor2_us = MOTOR_STOP + (M2_FWD - MOTOR_STOP) * mag / 100;
  }
}

void triggerBump(Phase resumePhase, int preferDir) {
  unsigned long now = millis();
  // Rapid re-stall detection: if we finished a bump less than
  // BUMP_REPEAT_WINDOW_MS ago, escalate. Otherwise reset the counter.
  if (bumpLastDoneAt != 0 && now - bumpLastDoneAt < BUMP_REPEAT_WINDOW_MS) {
    bumpRepeatCount++;
  } else {
    bumpRepeatCount = 0;
  }

  // Give up after repeated failures — the robot is probably wedged. Drop to
  // manual so the operator can intervene instead of thrashing forever.
  if (bumpRepeatCount >= BUMP_REPEAT_GIVEUP) {
    motorsOff();
    phase = PH_MANUAL;
    bumpState = BUMP_IDLE;
    bumpRepeatCount = 0;
    bumpLastDoneAt = 0;
    pushBle("BUMP_GIVEUP");
    SERIAL_LOG.println("BUMP: repeated re-stall, giving up → PH_MANUAL");
    return;
  }

  // Escalation schedule: each repeat backs up longer, flips turn direction,
  // and turns a bit further. First hit is gentle; wedged situations get
  // progressively more aggressive.
  bumpBackMs        = 800UL + 400UL * (unsigned long)bumpRepeatCount;
  bumpTargetAngle   = 120.0f + 45.0f * (float)bumpRepeatCount;
  bumpTurnTimeoutMs = 1500UL + 500UL * (unsigned long)bumpRepeatCount;

  bumpState = BUMP_BACKING;
  bumpEndTime = now + bumpBackMs;
  if (preferDir != 0) {
    bumpDir = preferDir;
  } else if (bumpRepeatCount > 0) {
    bumpDir = -bumpDir;  // re-stall: try turning the other way
  } else {
    bumpDir = random(2) ? 1 : -1;
  }
  bumpReturnPhase = resumePhase;
  motorsOff();
  stallPrevTime = 0;
  calTracking = false;
  seekProbing = false;       // invalidate any in-flight probe — baseline is stale
  SERIAL_LOG.printf("BUMP recovery #%d: back %lums, turn ~%.0f° %s\n",
                    (int)bumpRepeatCount, bumpBackMs, bumpTargetAngle,
                    bumpDir > 0 ? "CCW" : "CW");
}

bool handleBumpRecovery() {
  if (bumpState == BUMP_IDLE) return false;
  unsigned long now = millis();

  if (bumpState == BUMP_BACKING) {
    if (now < bumpEndTime) { driveBackward(); return true; }
    bumpState = BUMP_TURNING;
    bumpEndTime = now + bumpTurnTimeoutMs;
    bumpPrevHdg = readHeading();
    bumpCumTurn = 0;
  }

  if (bumpState == BUMP_TURNING) {
    if (imuOK) {
      float cur = readHeading();
      bumpCumTurn += headingDiff(bumpPrevHdg, cur);
      bumpPrevHdg = cur;
    }
    bool doneByAngle = imuOK && fabs(bumpCumTurn) >= bumpTargetAngle;
    bool doneByTime  = now >= bumpEndTime;
    if (doneByAngle || doneByTime) {
      bumpState = BUMP_IDLE;
      bumpLastDoneAt = now;
      motorsOff();
      SERIAL_LOG.printf("BUMP done (turn=%+.0f°, %s)\n", bumpCumTurn,
                        doneByAngle ? "angle" : "timeout");
      return false;
    }
    turnInPlace(bumpDir > 0 ? driveSpeedPct : -driveSpeedPct);
    return true;
  }
  return false;
}

// Accelerometer-based impact detection. Was previously disabled due to false
// triggers; now the sole collision detector when LEAN_MODE suspends UWB (and
// therefore position-based stall detection). Gated on direction-agnostic
// motion so it fires on forward, reverse, and counter-rotation spins alike.
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

void checkNoGo(Phase currentPhase) {
  if (noGoX1 == 0 && noGoY1 == 0 && noGoX2 == 0 && noGoY2 == 0) return;
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

void checkStall(Phase currentPhase) {
  // Need to be commanding forward motion AND have at least one UWB range
  // stream. We use whichever distance (to T0 or A0) changed more as the
  // motion proxy, so the check still works if one anchor is out of range or
  // if we happen to be moving tangent to one of them.
  if (!isDrivingForward()) { stallPrevTime = 0; return; }
  if (!kfAnc[0].init && !kfAnc[1].init) { stallPrevTime = 0; return; }

  unsigned long now = millis();
  float d_t0 = kfAnc[1].init ? kfAnc[1].x : 0.0f;
  float d_a0 = kfAnc[0].init ? kfAnc[0].x : 0.0f;

  if (stallPrevTime == 0) {
    stallPrevDT0 = d_t0;
    stallPrevDA0 = d_a0;
    stallPrevTime = now;
    return;
  }
  if (now - stallPrevTime < STALL_WINDOW_MS) return;

  float mvT0 = kfAnc[1].init ? fabsf(d_t0 - stallPrevDT0) : 0.0f;
  float mvA0 = kfAnc[0].init ? fabsf(d_a0 - stallPrevDA0) : 0.0f;
  float moved = fmaxf(mvT0, mvA0);

  if (moved < STALL_MIN_MOVE_CM) {
    SERIAL_LOG.printf("STALL: ΔdT0=%.0f ΔdA0=%.0f < %.0fcm in %lums → bump!\n",
                      mvT0, mvA0, STALL_MIN_MOVE_CM, STALL_WINDOW_MS);
    triggerBump(currentPhase, 0);
    stallPrevTime = 0;  // restart the window after recovery is initiated
  } else {
    stallPrevDT0 = d_t0;
    stallPrevDA0 = d_a0;
    stallPrevTime = now;
  }
}

void maybeAutoCalibrate(float drift) {
  if (!posValid || !imuOK || fabs(drift) > 25.0f) { calTracking = false; return; }
  unsigned long now = millis();
  if (!calTracking) { calPrevX = tagX; calPrevY = tagY; calPrevTime = now; calTracking = true; return; }
  if (now - calPrevTime < 800) return;
  float dx = tagX - calPrevX, dy = tagY - calPrevY;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 10.0f) {
    if (now - calPrevTime > 5000) { calPrevX = tagX; calPrevY = tagY; calPrevTime = now; }
    return;
  }
  // BNO055 is CCW-positive; atan2 is CCW-positive from +X. Invariant:
  //   compass = mapBasisHdg + α  →  mapBasisHdg = compass − α
  float mapAngle = atan2f(dy, dx) * 180.0f / M_PI;
  float compass = readHeading();
  float apparent = fmodf(compass - mapAngle + 360.0f, 360.0f);
  float diff = headingDiff(mapBasisHdg, apparent);
  mapBasisHdg = fmodf(mapBasisHdg + diff * 0.70f + 360.0f, 360.0f);
  SERIAL_LOG.printf("AUTOCAL d=%.0f mapAng=%+.0f comp=%.0f basis=>%.0f (corr=%+.1f)\n",
                    d, mapAngle, compass, mapBasisHdg, diff * 0.70f);
  calPrevX = tagX; calPrevY = tagY; calPrevTime = now;
  seekWrongCount = 0;
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

// ── Matrix animation engine (mirrored from T0) ───────────
//
// T1 has no MAX7219 LED matrix, but it does have a 128x64 SSD1306 OLED
// that was sitting unused. We reuse T0's exact 8x8 animation engine
// (BOUNCE / GOL / OVOID_MERGE / OVOID_HOLD), render the resulting 8x8
// dispBuf to the OLED scaled 8x to a centered 64x64 area, and expose
// pixel (0,0) as a new indicator-LED mode (see ledMode == 4 below).
//
// Each robot has its own RNG, so when both T0 and T1 run the same
// animation type they produce visually similar but pixel-for-pixel
// independent patterns. Pixel-perfect sync over BLE is intentionally
// out of scope for v1 (would need ~8 extra bytes per BLE push).

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
  0b00010000,
  0b00000000,
  0b00000000,
  0b10000001,
  0b10000001,
  0b00000000,
  0b00000000,
  0b00010000,
};

struct Bouncer { int x, y, dx, dy; unsigned long nextNudgeT; };
Bouncer bouncers[5];
uint8_t lifeBuf[8];
uint8_t dispBuf[8];     // current on-screen state — retained across frames
                        // so mode switches (especially → OVOID_MERGE) can
                        // start from whatever is currently shown.

static unsigned long randNudgeDelay() {
  return 1000UL + (unsigned long)random(2001);
}

#define RANDOMIZE_DIR(i) do { \
    do { bouncers[i].dx = (int)random(3) - 1; \
         bouncers[i].dy = (int)random(3) - 1; \
    } while (bouncers[i].dx == 0 && bouncers[i].dy == 0); \
  } while (0)

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

// Render the 8x8 dispBuf to the SSD1306 OLED. Each lit cell becomes an
// 8x8 filled rect; the 64x64 active area is centered on the 128x64 panel
// (32-px margin left and right). I2C transfer of the 1 KB framebuffer
// takes ~10 ms but is naturally throttled by matrixSpeedMs (50-500 ms
// typical), so no extra rate limiting needed.
void drawMatrixToOled() {
  if (!T1_OLED_RENDER) return;   // OLED off — see T1_OLED_RENDER at top
  display.clearDisplay();
  for (int y = 0; y < 8; y++) {
    uint8_t row = dispBuf[y];
    for (int x = 0; x < 8; x++) {
      if (row & (1 << x)) display.fillRect(32 + x * 8, y * 8, 8, 8, SSD1306_WHITE);
    }
  }
  display.display();
}

// Apply one mode change. Called from the 0x07 BLE handler, the SOP_ANIM_*
// autoplay ops, and the LED-GoL auto-arm path. Kept in one place so all
// three entry points stay consistent. speedByte is 5-50 in 10ms units.
void matrixSetMode(uint8_t mode, uint8_t cmplx, uint8_t speedByte) {
  matrixSpeedMs = (unsigned long)constrain((int)speedByte, 5, 50) * 10;
  switch (mode) {
    case MMODE_OFF:
      matrixMode = MMODE_OFF;
      matrixComplexity = 0;
      memset(dispBuf, 0, 8);
      drawMatrixToOled();
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
      matrixMode = MMODE_OVOID_MERGE;
      matrixComplexity = 0;
      break;
    default:
      break;
  }
}

// One animation frame. Dispatches by current mode; each mode produces the
// next dispBuf[] which we then push to the OLED via drawMatrixToOled().
// Called from loop() at line rate; internal throttling limits actual
// frame updates to matrixSpeedMs.
void matrixAnimTick() {
  // Engine always runs — costs <1 ms/frame and feeds dispBuf, which the
  // GoL-pixel LED mode (ledMode==4) reads regardless of OLED state.
  // drawMatrixToOled() below is the only path gated by T1_OLED_RENDER.
  if (matrixMode == MMODE_OFF || matrixMode == MMODE_OVOID_HOLD) return;
  if (matrixSpeedMs == 0) return;

  const unsigned long now = millis();
  if (now - lastMatrixFrame < matrixSpeedMs) return;
  lastMatrixFrame = now;

  if (matrixMode == MMODE_BOUNCE || matrixMode == MMODE_GOL) {
    int nb = (matrixMode == MMODE_BOUNCE)
             ? constrain((int)matrixComplexity, 1, 5)
             : 5;

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
      for (int i = 0; i < nb; i++)
        lifeBuf[bouncers[i].y] |= (1 << bouncers[i].x);

      if (matrixComplexity >= 7) {
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
      dispBuf[y] = cur | target;
    }
    if (!anyExtra) matrixMode = MMODE_OVOID_HOLD;
  }

  drawMatrixToOled();
}

// ── Status LED ───────────────────────────────────────────
//
// Two layers of control:
//   1. ledSolid() / ledOff() / ledBlink() — legacy, called from the
//      seek / bump / wait state machines to signal phase-internal state.
//   2. ledMode / tickLedMode() — commanded directly from the browser via
//      command 0x12. Modes: 0=off, 1=solid, 2=slow-blink, 3=fast-blink,
//      4=GoL pixel (LED reflects dispBuf[0] & 1 of the matrix engine).
//      Runs in parallel with the legacy helpers. In LEAN_MODE (no
//      seek / no wait phases running), only this path drives the LED, so
//      blink patterns come through undisturbed.

uint8_t ledMode = 1;                 // 0 off / 1 solid / 2 slow / 3 fast / 4 gol-pixel / 5 random
unsigned long ledNextToggleMs = 0;
bool          ledToggleState   = false;

void ledSolid()   { digitalWrite(STATUS_LED, HIGH); }
void ledOff()     { digitalWrite(STATUS_LED, LOW); }

void ledBlink() {
  static unsigned long last = 0;
  static bool on = false;
  if (millis() - last > 250) { last = millis(); on = !on; digitalWrite(STATUS_LED, on); }
}

// Non-blocking indicator-LED ticker. Called every loop() iteration from
// MANUAL phase (which is where LEAN_MODE spends 100% of its time).
void tickLedMode() {
  unsigned long now = millis();
  switch (ledMode) {
    case 0:  digitalWrite(STATUS_LED, LOW);  break;
    case 1:  digitalWrite(STATUS_LED, HIGH); break;
    case 2: {  // slow blink: 1 Hz = 500 ms per edge
      if ((long)(now - ledNextToggleMs) >= 0) {
        ledToggleState = !ledToggleState;
        digitalWrite(STATUS_LED, ledToggleState);
        ledNextToggleMs = now + 500;
      }
      break;
    }
    case 3: {  // fast blink: 4 Hz = 125 ms per edge
      if ((long)(now - ledNextToggleMs) >= 0) {
        ledToggleState = !ledToggleState;
        digitalWrite(STATUS_LED, ledToggleState);
        ledNextToggleMs = now + 125;
      }
      break;
    }
    case 4:  // GoL pixel — track (0,0) of the matrix engine's dispBuf.
             // Updated at line rate; the underlying engine throttles its
             // own evolution to matrixSpeedMs (default 200 ms/frame).
      digitalWrite(STATUS_LED, (dispBuf[0] & 1) ? HIGH : LOW);
      break;
    case 5: {  // random: alternate ON/OFF, each phase 100..1000 ms picked
               // independently. Reuses ledNextToggleMs as the deadline and
               // ledToggleState as the current LED state.
      if ((long)(now - ledNextToggleMs) >= 0) {
        ledToggleState  = !ledToggleState;
        digitalWrite(STATUS_LED, ledToggleState);
        // 100..1000 ms inclusive (random(min,max) is upper-exclusive).
        ledNextToggleMs = now + (unsigned long)random(100, 1001);
      }
      break;
    }
  }
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

// Forward decls for the autoplay state accessors used below. Bodies live
// alongside the autoplay runner state (with #else stubs for AUTOPLAY=0).
uint8_t  autoplayMotorIdxByte();
uint8_t  autoplayAnimIdxByte();
uint16_t autoplayMotorMsLeft();

// stop()/start() around setAdvertisementData() is required on this BLE stack — without
// it the new payload is sometimes not actually published. The old delay(10) is gone
// and the 1 Hz throttle in checkBleEvents() keeps radio current down.
//
// Manufacturer-data layout matches the emitter (see mauwb_emitter.ino for the full
// table): 14 bytes total, with bytes 8..11 carrying motor/anim indices and motor
// ms-until-next for sync, and bytes 12..13 carrying free heap / 32 (BE) so the
// anchor can relay heap health to the web UI. T1 (this robot) publishes its own
// runner state for monitoring/symmetry but is the *follower* — only T0's bytes
// 8..11 are acted on (in PeerScanCB → syncToPeer()).
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
                 | 0x80;   // tid = 1
  uint8_t hdg = (uint8_t)(readHeading() / 2.0);

  uint8_t  syncMotor = autoplayMotorIdxByte();
  uint8_t  syncAnim  = autoplayAnimIdxByte();
  uint16_t syncMs    = autoplayMotorMsLeft();

  uint32_t freeHeap   = ESP.getFreeHeap();
  uint16_t heapUnits  = (uint16_t)constrain((int)(freeHeap >> 5), 0, 65535);

  BLEAdvertisementData ad;
  ad.setName("RobotB-Rx");
  ad.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  uint8_t mf[14] = { 0xFF, 0xFF,
    (uint8_t)(d0 >> 8), (uint8_t)(d0 & 0xFF),
    (uint8_t)(d1 >> 8), (uint8_t)(d1 & 0xFF),
    status, hdg,
    syncMotor, syncAnim,
    (uint8_t)(syncMs >> 8),    (uint8_t)(syncMs & 0xFF),
    (uint8_t)(heapUnits >> 8), (uint8_t)(heapUnits & 0xFF) };
  ad.setManufacturerData(String((char*)mf, 14));

  // In-place update avoids the stop/delay/start radio transient (brownout mitigation).
  // Length is fixed at 14 bytes; first push from BOOT establishes that.
  pBleAdv->setAdvertisementData(ad);

  lastBlePush  = millis();
  lastBleDist  = (uint8_t)constrain((int)d0, 0, 255);
  lastBlePhase = (uint8_t)phase;

  SERIAL_LOG.printf("BLE>> %s  d0=%d d1=%d phase=%d  motor=%u anim=%u ms=%u\n",
                    reason, d0, d1, (int)phase,
                    (unsigned)syncMotor, (unsigned)syncAnim, (unsigned)syncMs);
}

// Raised 1 Hz → 0.5 Hz to lower peak current (brownout mitigation).
const unsigned long BLE_MIN_INTERVAL_MS = 2000;

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
// cursors. Motor track is primary: its SOP_END ends the whole session.
// Keep in sync with mauwb_emitter.ino — ops are shared across robots.
//
// p1/p2 meanings:
//   motor track:
//     SOP_FWD/REV/TL/TR/WALK  p1 = speed %          p2 = 0
//     SOP_PAUSE               p1 = 0                p2 = 0
//   anim track:
//     SOP_ANIM_*              emitter-only; receiver silently ignores
//     SOP_LED                 p1 = mode (0..3)      p2 = 0
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
// AutoSeq pairs the two tracks. Matches the emitter so the shared
// autoplay_sequences.h compiles identically on both robots.
struct AutoSeq  { const AutoStep* motor; const AutoStep* anim; };

#if AUTOPLAY

// Pulls in SEQ_*_MOTOR[], SEQ_*_ANIM[], AUTOPLAY_TABLE[] (array of AutoSeq),
// AUTOPLAY_NAMES[], and AUTOPLAY_COUNT. Regenerate from
// position_map/index.html (Test Sequence card → "Export fw").
#include "autoplay_sequences.h"

// Motor track runner: primary. SOP_END here ends the whole autoplay run.
unsigned long   autoplayStartAt    = 0;
bool            autoplayActive     = false;
const AutoStep* autoMotor          = nullptr;
uint16_t        autoMotorIdx       = 0;
unsigned long   autoMotorStepEnd   = 0;
uint16_t        autoRepeatDone     = 0;

// Anim track runner: secondary. SOP_END just stops this track; we keep
// ticking motor to completion.
const AutoStep* autoAnim           = nullptr;
uint16_t        autoAnimIdx        = 0;
unsigned long   autoAnimStepEnd    = 0;

// State accessors used by pushBle() to publish runner state for inter-
// robot sequence sync. Returns 0xFF when the corresponding track is idle.
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
  // Solid-on indicator to mark "running" (overrides the slow-blink arm
  // state). User-commanded ledMode still wins after autoplay finishes.
  ledMode = 1;
  SERIAL_LOG.printf("AUTOPLAY begin: seq=%d \"%s\" repeat=%d\n",
                    seqIdx, AUTOPLAY_NAMES[seqIdx], AUTOPLAY_REPEAT);
}

void autoplayTick() {
  if (!autoplayActive) return;
  unsigned long now = millis();

  // ── Motor track (primary) ──
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
      bool keepGoing = (AUTOPLAY_REPEAT == 0) || (autoRepeatDone < AUTOPLAY_REPEAT);
      if (keepGoing) {
        SERIAL_LOG.printf("AUTOPLAY repeat %u\n", (unsigned)autoRepeatDone + 1);
        autoMotorIdx     = 0;
        autoAnimIdx      = 0;
        autoAnimStepEnd  = 0;
        motorsOff(); walkActive = false;
        autoMotorStepEnd = now + 150;
        return;
      }
      motorsOff(); walkActive = false;
      autoplayActive = false;
      SERIAL_LOG.println("AUTOPLAY complete");
      return;
    }

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
    break;
  }

  // ── Anim track (secondary) ──
  // SOP_ANIM_* drive T1's matrix engine (rendered to the OLED, see
  // matrixSetMode/matrixAnimTick above). SOP_LED drives the indicator
  // LED. When the anim track hits SOP_END it just stops advancing; the
  // motor track continues to completion. Same target-skip semantics as
  // the motor track above.
  while (autoAnim && now >= autoAnimStepEnd) {
    uint8_t  op     = autoAnim[autoAnimIdx].op;
    uint8_t  p1     = autoAnim[autoAnimIdx].p1;
    uint8_t  p2     = autoAnim[autoAnimIdx].p2;
    uint8_t  target = autoAnim[autoAnimIdx].target;
    uint32_t dur_ms = autoAnim[autoAnimIdx].dur_ms;

    if (op == SOP_END) {
      autoAnim = nullptr;
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
      case SOP_LED: {
        uint8_t newMode = (p1 <= 5) ? p1 : 0;
        // Mode 4 = "GoL pixel" — auto-arm matrix engine to GoL so the
        // LED has something interesting to tap. If the engine is already
        // in GoL we leave it alone (don't reseed mid-evolution).
        // Mode 5 = "random" — needs nothing else; ledMode tick handles it.
        if (newMode == 4 && matrixMode != MMODE_GOL) {
          matrixSetMode(MMODE_GOL, 7, 20);
        }
        ledMode = newMode;
        break;
      }
      default: break;
    }
    autoAnimStepEnd = now + dur_ms;
    autoAnimIdx++;
    break;
  }
}

// Indicator-LED countdown during the AUTOPLAY_DELAY_MS window. Fast blink
// (~4 Hz) is distinct from the steady or slow modes the user might
// otherwise see. Stops on its own when autoplay begins.
void autoplayCountdownTick() {
  static unsigned long lastTick = 0;
  static bool lit = false;
  unsigned long now = millis();
  if (now - lastTick < 250) return;
  lastTick = now;
  lit = !lit;
  // Temporarily override ledMode via direct pin write. We use mode 0 (off)
  // as a scratch state; tickLedMode() will repaint once autoplay starts.
  ledMode = lit ? 1 : 0;
}

// ── Inter-robot sync (T1 → T0) ───────────────────────────
// T0 is the leader. Whenever we get a fresh peer advertisement we either
// (a) fast-forward our motor track if T0 is ahead by ≥1 step, or
// (b) slide our deadline if we're on the same step but >SYNC_SLEW_MS
//     out of phase.
// We never roll back, so if T1 happens to be ahead of T0 we simply ride
// it out — both runners will re-converge at the next step boundary. To
// damp jitter, we throttle nudges to SYNC_INTERVAL_MS and ignore anything
// older than SYNC_FRESH_MS (peer rebooted, out of range, etc.).
const unsigned long SYNC_FRESH_MS    = 1500;
const long          SYNC_SLEW_MS     = 200;
const unsigned long SYNC_INTERVAL_MS = 250;

unsigned long lastSyncNudge = 0;

void syncToPeer() {
  if (!autoplayActive || !autoMotor) return;
  if (!peerSyncValid) return;

  unsigned long now = millis();
  if (now - lastSyncNudge < SYNC_INTERVAL_MS) return;

  // Snapshot volatile state once so a concurrent ad arrival mid-function
  // can't tear our comparison.
  uint8_t       pMotor  = peerMotorIdx;
  uint16_t      pMsLeft = peerMotorMsLeft;
  unsigned long pRxMs   = peerSyncRxMs;
  if (pMotor == 0xFF) return;                    // peer not running yet
  if (now - pRxMs > SYNC_FRESH_MS) return;       // peer info stale

  // Account for transit time: if the ad was received 200 ms ago, peer's
  // remaining time is now 200 ms less than what they reported.
  unsigned long age   = now - pRxMs;
  uint16_t pMsLeftAdj = (pMsLeft > age) ? (uint16_t)(pMsLeft - age) : 0;

  uint16_t myIdx = (uint16_t)autoMotorIdx;
  if ((uint16_t)pMotor > myIdx) {
    // T0 is ahead of us by ≥1 step. Force our current step to end now;
    // autoplayTick() will dispatch the next step on the same loop pass.
    // For ≥2 step gaps, this nudge fires repeatedly (every
    // SYNC_INTERVAL_MS) until we catch up, which is plenty fast in
    // practice — typical drift is sub-step.
    autoMotorStepEnd = now;
    lastSyncNudge = now;
    SERIAL_LOG.printf("SYNC: catch-up motor %u→%u\n",
                      (unsigned)myIdx, (unsigned)pMotor);
    return;
  }

  if ((uint16_t)pMotor == myIdx) {
    unsigned long myLeft = (autoMotorStepEnd > now) ? autoMotorStepEnd - now : 0;
    long diff = (long)pMsLeftAdj - (long)myLeft;
    if (diff > SYNC_SLEW_MS || diff < -SYNC_SLEW_MS) {
      // Slide our deadline. Positive diff (peer has more time left)
      // means we ran ahead → push end out. Negative (peer is closer to
      // done) means we lagged → pull end in.
      autoMotorStepEnd = now + pMsLeftAdj;
      lastSyncNudge = now;
      SERIAL_LOG.printf("SYNC: slew motor=%u  diff=%+ldms  myLeft=%lu→%u\n",
                        (unsigned)myIdx, diff, myLeft, (unsigned)pMsLeftAdj);
    }
    return;
  }

  // pMotor < myIdx → we're ahead. Don't roll back; T0 will close the gap
  // on its own and the next equal-idx tick will re-align timing.
}
#else
// Stubs so pushBle() compiles when autoplay is off — no runner means
// "idle" on both tracks. syncToPeer() has no caller in this branch.
uint8_t  autoplayMotorIdxByte() { return 0xFF; }
uint8_t  autoplayAnimIdxByte()  { return 0xFF; }
uint16_t autoplayMotorMsLeft()  { return 0; }
#endif  // AUTOPLAY

// ── Setup ────────────────────────────────────────────────

void setup() {
  pinMode(RESET_PIN, OUTPUT);
#if LEAN_MODE
  // Hold the DW3000 STM32 in reset so the UWB front-end draws ~0 mA.
  // This is a bigger saving than just skipping AT init: the STM32 would
  // otherwise come up in whatever role was last saved to its flash and
  // start ranging / responding on its own.
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

  SERIAL_LOG.println("=== Robot-B Receiver v4 ===");
  SERIAL_LOG.printf("Trim-only: gain=%.1f max=%d deadband=%.1f\n", TRIM_GAIN, TRIM_MAX, DEADBAND_DEG);

  // IR LED always on (gated so it can be dropped for brownout relief)
  pinMode(IR_LED_PIN, OUTPUT);
#if ENABLE_IR_LED
  digitalWrite(IR_LED_PIN, HIGH);
#else
  digitalWrite(IR_LED_PIN, LOW);
  SERIAL_LOG.println("IR LED: disabled (ENABLE_IR_LED=0)");
#endif

  // Status LED on during init
  pinMode(STATUS_LED, OUTPUT);
  ledSolid();

  // DFPlayer init is a brownout-risky sequence (SD card mount + amp bias
  // over ~500 ms). Skip it unless audio is actually wired up.
#if ENABLE_DFPLAYER
  Serial1.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(500);
  if (dfPlayer.begin(Serial1)) { dfPlayerOK = true; dfPlayer.volume(30); }
  SERIAL_LOG.printf("DFPlayer: %s\n", dfPlayerOK ? "OK" : "FAIL");
#else
  SERIAL_LOG.println("DFPlayer: disabled (ENABLE_DFPLAYER=0)");
#endif

  // BLE + GATT command service (low TX power, low scan duty → low average current).
  BLEDevice::init("RobotB-Rx");
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
  ad.setName("RobotB-Rx");
  ad.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  pBleAdv->setAdvertisementData(ad);
  BLEAdvertisementData scanResp;
  scanResp.setName("RobotB-Rx");
  pBleAdv->setScanResponseData(scanResp);
  pBleAdv->start();
  SERIAL_LOG.printf("BLE: MAC=%s\n", BLEDevice::getAddress().toString().c_str());

  pBleScan = BLEDevice::getScan();
  pBleScan->setAdvertisedDeviceCallbacks(new PeerScanCB(), true);
  pBleScan->setActiveScan(false);
  // 10 ms scan every 100 ms (10% duty, 10 Hz). See rationale in
  // mauwb_emitter.ino — the previous 5% @ 1 Hz sampled too slowly to
  // reliably catch per-second command changes during test sequences.
  // Keep this in sync with the emitter's scan settings.
  pBleScan->setInterval(160);   // 100 ms cycle
  pBleScan->setWindow(16);      //  10 ms window
  pBleScan->start(0, nullptr, false);
  delay(500);

  SERIAL_LOG.printf("HEAP: post-BLE-init free=%u (baseline for leak watch)\n",
                    (unsigned)ESP.getFreeHeap());

  // BNO055 IMU — skipped in LEAN_MODE unless ENABLE_BUMP_DETECT keeps it
  // alive for impact detection. Without IMU, forwardWithTrim() naturally
  // falls back to raw driveForward() (see the !imuOK branch); sequence
  // playback doesn't need heading trim.
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

  // UWB Tag 1 config — skipped in LEAN_MODE (RESET held LOW above). When
  // re-enabled, ranges to fixed anchors for trilateration.
#if !LEAN_MODE
  sendAT("AT+RESTORE", 5000);
  sendAT("AT+SETCFG=1,0,1,1", 2000);
  sendAT("AT+SETCAP=10,10,1", 2000);
  sendAT("AT+SETRPT=1", 2000);
  // Antenna-delay calibration — MUST match the value set in mauwb_emitter.ino
  // (both ends of the UWB link contribute equally to the reported distance).
  // See the extended comment in mauwb_emitter.ino for tuning procedure.
  sendAT("AT+SETANT=16493", 2000);
  sendAT("AT+SAVE", 2000);
  sendAT("AT+RESTART", 2000);
#else
  SERIAL_LOG.println("UWB:    disabled (LEAN_MODE=1)");
#endif

  phase = PH_MANUAL;
  pushBle("BOOT");
  SERIAL_LOG.println("Starting in MANUAL mode (motors off)");

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
  // Matrix engine ticks unconditionally so the OLED keeps animating in
  // every phase (MANUAL early-returns at the bottom; without this hook
  // the OLED would freeze whenever the receiver wasn't in PH_MANUAL).
  // Internal throttling caps work at one frame per matrixSpeedMs.
  matrixAnimTick();

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
  if (autoplayStartAt > 0) {
    autoplayCountdownTick();
    if (millis() >= autoplayStartAt) {
      autoplayStartAt = 0;
      autoplayBegin(AUTOPLAY_SEQUENCE);
    }
  }
  // Sync first so a catch-up nudge (autoMotorStepEnd = now) can be
  // dispatched by the same autoplayTick() call on this loop pass.
  syncToPeer();
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
      // No-go obstacle line: 4 × int16 big-endian cm = x1,y1,x2,y2.
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
      // Path: count (0..MAX_PATH_WP) + N × (xhi xlo yhi ylo) int16 BE. count=0 clears.
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
          seekProbeArmed = true;   // auto-calibrate before navigating waypoints
          seekProbing = false;
          pushBle("PATH");
          SERIAL_LOG.printf("PATH set: %d wp, first=(%d,%d)\n", pathLen, pathWp[0].x, pathWp[0].y);
        }
      }
    } else if (ct == 0x10) {
      // Distance calibration offset: int16 big-endian, cm. Applied to raw
      // d(T0,T1) before SEEK arrival / rate comparisons.
      if (len >= 3) {
        distOffsetCm = (int16_t)(((uint16_t)(uint8_t)buf[1] << 8) | (uint8_t)buf[2]);
        SERIAL_LOG.printf("DIST offset = %d cm\n", (int)distOffsetCm);
      }
    } else if (ct == 0x0F) {
      // SET POSE: payload = xhi xlo yhi ylo hhi hlo (3× int16 BE, cm / degrees).
      // Seeds pos + mapBasisHdg — use via click-drag on the map.
      if (len >= 7) {
        int16_t px = (int16_t)(((uint16_t)(uint8_t)buf[1] << 8) | (uint8_t)buf[2]);
        int16_t py = (int16_t)(((uint16_t)(uint8_t)buf[3] << 8) | (uint8_t)buf[4]);
        int16_t ph = (int16_t)(((uint16_t)(uint8_t)buf[5] << 8) | (uint8_t)buf[6]);
        tagX = (float)px; tagY = (float)py;
        posValid = true;
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
        seekProbeArmed = false; seekProbing = false;
        pushBle("POSE");
        SERIAL_LOG.printf("POSE set: (%d,%d) hdg=%d° → basis=%.0f\n", px, py, ph, mapBasisHdg);
      }
    } else if (ct == 0x05) {
      motorsOff(); walkActive = false;
      phase = PH_SEEK;
      calTracking = false;
      stallPrevTime = 0;
      seekOvershootUntil = 0;
      bumpState = BUMP_IDLE;
      bumpRepeatCount = 0;    // fresh seek, fresh repeat counter
      bumpLastDoneAt = 0;
      // Distance-gradient seek: forward probe, then hill-climb. Initial
      // turn direction is arbitrary; rate comparison corrects it after
      // the first turn.
      seekStep = SS_DRIVE;
      seekStepStart = 0;   // latched on first tick inside SS_DRIVE
      seekStartDist = 0;
      seekStartDistA0 = 0;
      seekWrongCount = 0;
      seekTurnDir = (random(2) ? +1 : -1);  // random start avoids always-right bias
      seekPrevDelta = 0;
      seekHasPrevDelta = false;
      seekJustTurned = false;
      seekStallStreak = 0;
      // Legacy trilat-based probe no longer used (T0 is mobile, so the old
      // "drive 25 cm to find the map basis" dance is nonsense now).
      seekProbeArmed = false;
      seekProbing = false;
      pushBle("SEEK");
      SERIAL_LOG.printf("SEEK mode (rate-comparison hill climb, distOffset=%dcm)\n",
                        (int)distOffsetCm);
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
      // 10-second arm window aborts boot-time playback. Matrix/OLED is
      // parked off so the OFF state is visually unambiguous.
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
    } else if (ct == 0x03) {
      // Legacy direct on/off. Treated as modes 0 / 1 so the mode-based
      // ticker stays authoritative; otherwise tickLedMode() would
      // immediately overwrite the digitalWrite below.
      ledMode = p1 ? 1 : 0;
      ledNextToggleMs = 0;
    } else if (ct == 0x12) {
      // Indicator LED mode: 0=off, 1=solid, 2=slow-blink (1 Hz),
      // 3=fast-blink (4 Hz), 4=GoL pixel (LED reflects dispBuf[0] & 1
      // of the matrix engine), 5=random (random ON/OFF phases of
      // 100..1000 ms). Mode 4 auto-arms matrix-GoL if it isn't already
      // running so the LED has something to tap.
      uint8_t newMode = (p1 <= 5) ? p1 : 0;
      if (newMode == 4 && matrixMode != MMODE_GOL) {
        matrixSetMode(MMODE_GOL, 7, 20);
      }
      ledMode = newMode;
      ledNextToggleMs = 0;
      ledToggleState  = false;
      SERIAL_LOG.printf("LED mode = %u\n", (unsigned)ledMode);
    } else if (ct == 0x04 && dfPlayerOK) {
      dfPlayer.play(constrain(p1, 1, 32));
    } else if (ct == 0x06) {
      mapBasisHdg = readHeading(); targetHdg = mapBasisHdg;
      SERIAL_LOG.printf("Calibrate: mapBasis=%.1f\n", mapBasisHdg);
    } else if (ct == 0x07) {
      // p1 packs (mode << 4) | cmplx. Modes:
      //   0 = off, 1 = bouncers, 2 = GoL, 3 = ovoid-merge.
      // p2 = anim speed (5-50, 10 ms units).
      // Drives the matrix engine, rendered to the OLED. Same wire format
      // as the emitter so a single 0x07 command updates both robots.
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
      // Explicit walk-stop: cancel walk without changing phase
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
    tickLedMode();
    checkBleEvents();
    return;
  }

  // ── WAIT: until first UWB reading ──
  if (phase == PH_WAIT_UWB) {
    ledBlink();
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
    ledSolid();

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
      arrivedTime = millis();
      audioPlayed = false;
      ledOff();
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
      SERIAL_LOG.printf("DRIVE  dist=%.0f  hdg=%.0f  drift=%+.1f  m1=%d m2=%d\n",
        kf_x, readHeading(), drift, motor1_us, motor2_us);
    }
    checkBleEvents();
    return;
  }

  // ── SEEK: drive toward T0 using only the measured d(T0,T1) ──
  // This is a simple distance-gradient seek: drive forward for a fixed
  // segment, check whether d(T0,T1) dropped; if not, turn and try again.
  // It doesn't need 2D position or heading calibration — just the one
  // ranged distance — so it works with the current topology (T0 is the
  // mobile UWB anchor that T1 ranges to).
  if (phase == PH_SEEK) {
    if (handleBumpRecovery()) { checkBleEvents(); return; }
    checkAccelBump(PH_SEEK);
    checkNoGo(PH_SEEK);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); checkBleEvents(); return; }

    // UWB-loss recovery: if we've lost the ranging stream, stop and wait.
    if (kfAnc[1].init && millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
      SERIAL_LOG.println("SEEK: UWB lost, waiting for fix...");
    }

    // We need at least one range to T0 (kfAnc[1]) before doing anything.
    // kfAnc[0] (A0) is nice to have but not required for this algorithm.
    if (!kfAnc[1].init) {
      motorsOff();
      ledBlink();
      if (millis() - lastLog >= 1000) {
        lastLog = millis();
        SERIAL_LOG.println("SEEK wait: no distance to T0");
      }
      checkBleEvents();
      return;
    }

    ledSolid();
    // Calibrated inter-robot distance. Raw readings are what BLE broadcasts
    // (so the browser can do its own display math), but everything that
    // drives seek behaviour — arrival, progress, rate comparison — runs on
    // this offset-corrected value so "arrived" matches the operator's zero.
    float dist = kfAnc[1].x - (float)distOffsetCm;

    // Arrival: once d(T0,T1) drops below SEEK_ARRIVE_DIST, commit the last
    // hand-span with a short blind forward drive (UWB accuracy is ~10-20 cm
    // so ranging alone can't close the final gap).
    if (seekOvershootUntil == 0 && dist < SEEK_ARRIVE_DIST) {
      unsigned long ms = 33333UL / (unsigned long)driveSpeedPct;
      if (ms < 200)  ms = 200;
      if (ms > 3000) ms = 3000;
      seekOvershootUntil = millis() + ms;
      SERIAL_LOG.printf("SEEK arrive d=%.0fcm, overshoot %lums\n", dist, ms);
    }
    if (seekOvershootUntil) {
      if (millis() >= seekOvershootUntil) {
        motorsOff();
        phase = PH_ARRIVED;
        seekOvershootUntil = 0;
        arrivedTime = millis();
        audioPlayed = false;
        ledOff();
        pushBle("ARRIVED");
        SERIAL_LOG.printf("MET! final d=%.0fcm\n", dist);
        checkBleEvents();
        return;
      }
      driveForward();
      checkStall(PH_SEEK);
      checkBleEvents();
      return;
    }

    // ── Drive segment ───────────────────────────────────────
    if (seekStep == SS_DRIVE) {
      if (seekStepStart == 0) {
        // First tick in this drive segment: latch the starting distances so
        // we can measure Δd at the end of the segment.
        seekStepStart = millis();
        seekStartDist   = dist;
        seekStartDistA0 = kfAnc[0].init ? kfAnc[0].x : 0;
      }
      // IMU trim keeps each forward leg reasonably straight despite motor
      // asymmetry. Without IMU we fall back to raw motor commands.
      if (imuOK) forwardWithTrim(0); else driveForward();
      if (millis() - seekStepStart >= SEEK_DRIVE_MS) {
        float delta   = dist - seekStartDist;  // negative = got closer to T0
        float deltaA0 = kfAnc[0].init ? kfAnc[0].x - seekStartDistA0 : 0;
        motorsOff();

        // Stall detection: d(T0,T1) barely moved over the segment. The old
        // A0-distance corroboration is vestigial — A0's UWB chipset is
        // offline now to prevent anchor-id collision with T0. When kfAnc[0]
        // is uninitialised (the normal case post-A0-removal), stuckA0 is
        // forced true and the gate collapses to T0-only. Still require TWO
        // consecutive stalled segments before firing bump — a single segment
        // can read near-zero Δd just from UWB noise or if T1 happens to be
        // moving tangent to T0 at the moment of sampling.
        bool stuckT0 = fabsf(delta)   < STALL_MIN_MOVE_CM;
        bool stuckA0 = !kfAnc[0].init || fabsf(deltaA0) < STALL_MIN_MOVE_CM;
        if (stuckT0 && stuckA0) {
          seekStallStreak++;
          SERIAL_LOG.printf("SEEK stall streak=%d (Δd_T0=%+.1f Δd_A0=%+.1f)\n",
                            (int)seekStallStreak, delta, deltaA0);
          if (seekStallStreak >= 2) {
            SERIAL_LOG.println("SEEK stall confirmed → bump");
            triggerBump(PH_SEEK, 0);
            seekStallStreak = 0;
            // Rate history is now invalid — baseline was taken against an
            // obstacle, not free motion. Clear it so post-recovery driving
            // doesn't trigger a bogus direction flip.
            seekHasPrevDelta = false;
            seekJustTurned = false;
            checkBleEvents();
            return;
          }
        } else {
          seekStallStreak = 0;
        }

        SERIAL_LOG.printf("SEEK drive %lums Δd=%+.1f d=%.0f(raw %.0f) prev=%+.1f turn=%+d %s\n",
                          (unsigned long)SEEK_DRIVE_MS, delta,
                          dist, kfAnc[1].x,
                          seekHasPrevDelta ? seekPrevDelta : 0.0f,
                          (int)seekTurnDir,
                          seekJustTurned ? "post-turn" : "straight");

        // Rate comparison: if the PREVIOUS action was a turn, use the change
        // in Δd to decide whether that turn was beneficial.
        //   delta < prevDelta − hyst  → closing rate improved, KEEP direction
        //   delta > prevDelta + hyst  → closing rate got worse, FLIP direction
        //   else                      → ambiguous, keep committed direction
        if (seekJustTurned && seekHasPrevDelta) {
          if (delta > seekPrevDelta + SEEK_RATE_HYST_CM) {
            int8_t old = seekTurnDir;
            seekTurnDir = -seekTurnDir;
            SERIAL_LOG.printf("SEEK rate worse (%+.1f → %+.1f) — flip turn %+d → %+d\n",
                              seekPrevDelta, delta, (int)old, (int)seekTurnDir);
          } else if (delta < seekPrevDelta - SEEK_RATE_HYST_CM) {
            SERIAL_LOG.printf("SEEK rate better (%+.1f → %+.1f) — keep turn %+d\n",
                              seekPrevDelta, delta, (int)seekTurnDir);
          }
        }

        seekPrevDelta    = delta;
        seekHasPrevDelta = true;

        if (delta < -SEEK_PROGRESS_CM) {
          // Closing distance — carry on straight, relatch start for next seg.
          seekWrongCount = 0;
          seekStartDist   = dist;
          seekStartDistA0 = kfAnc[0].init ? kfAnc[0].x : 0;
          seekStepStart = millis();
          seekJustTurned = false;
          // stay in SS_DRIVE
        } else {
          // No (or not enough) progress → turn and retry. Committed direction
          // may have been flipped above by the rate comparison.
          seekWrongCount++;
          seekStep = SS_TURN;
          seekStepStart = millis();
          seekJustTurned = true;
        }
      }
      // Stall detection for seek is handled inline above (seekStallStreak)
      // using the same UWB-delta idea — running checkStall() here in
      // parallel would double-fire with a different window/threshold.
      checkBleEvents();
      return;
    }

    // ── Turn segment ────────────────────────────────────────
    if (seekStep == SS_TURN) {
      // Fixed turn chunk, committed direction. The probe/evaluate after each
      // chunk means if we pass through the correct bearing we'll detect it
      // on the next SS_DRIVE and stop turning further.
      turnInPlace(seekTurnDir > 0 ? (int8_t)driveSpeedPct : -(int8_t)driveSpeedPct);
      if (millis() - seekStepStart >= SEEK_TURN_MS) {
        motorsOff();
        SERIAL_LOG.printf("SEEK turn %lums dir=%+d sweep=%d d=%.0f\n",
                          SEEK_TURN_MS, (int)seekTurnDir,
                          (int)seekWrongCount, dist);
        // If we've racked up SEEK_SWEEP_FULL consecutive no-progress chunks,
        // we've likely swept past 360° without finding a closing heading.
        // Flip direction once as a fallback (maybe T0 has moved) and reset
        // the counter.
        if (seekWrongCount >= SEEK_SWEEP_FULL) {
          seekTurnDir  = -seekTurnDir;
          seekWrongCount = 0;
          SERIAL_LOG.printf("SEEK full sweep failed — reverse turn dir → %+d\n",
                            (int)seekTurnDir);
        }
        seekStep = SS_DRIVE;
        seekStepStart = 0;  // relatch on next tick
      }
      checkBleEvents();
      return;
    }

    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("SEEK d=%.0f step=%s wrong=%d\n",
        dist, seekStep == SS_DRIVE ? "DRIVE" : "TURN", (int)seekWrongCount);
    }
    checkBleEvents();
    return;
  }

  // ── PATH: follow user-drawn waypoints ──────────────────
  if (phase == PH_PATH) {
    ledSolid();
    if (handleBumpRecovery()) { checkBleEvents(); return; }
    checkAccelBump(PH_PATH);
    checkNoGo(PH_PATH);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); checkBleEvents(); return; }

    if (kfInit && millis() - lastUwbTime > UWB_TIMEOUT_MS) {
      motorsOff();
      kfAnc[0].reset(); kfAnc[1].reset(); kfInit = false; posValid = false;
      pushBle("UWB_LOST");
    }
    if (!posValid) { motorsOff(); checkBleEvents(); return; }

    // Motion-probe auto-calibration (same as SEEK).
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
      arrivedTime = millis();
      audioPlayed = false;
      ledOff();
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
    ledSolid();
    if (handleBumpRecovery()) { checkBleEvents(); return; }
    checkAccelBump(PH_WANDER);
    checkNoGo(PH_WANDER);
    if (bumpState != BUMP_IDLE) { handleBumpRecovery(); checkBleEvents(); return; }

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

    if (millis() - lastLog >= 500) {
      lastLog = millis();
      SERIAL_LOG.printf("WANDER hdg=%.0f drift=%+.1f pos=(%.0f,%.0f) basis=%.0f\n",
        wanderHdg, drift, tagX, tagY, mapBasisHdg);
    }
    checkBleEvents();
    return;
  }

  // ── ARRIVED: stay stopped, play audio after 5s ──
  if (phase == PH_ARRIVED) {
    motorsOff();

    static unsigned long arrvBlink = 0;
    static bool arrvOn = false;
    if (millis() - arrvBlink > 1000) { arrvBlink = millis(); arrvOn = !arrvOn; digitalWrite(STATUS_LED, arrvOn); }

    if (millis() - lastLog >= 2000) {
      lastLog = millis();
      SERIAL_LOG.printf("ARRIVED  dist=%.0f\n", kf_x);
    }
    checkBleEvents();
  }
}
