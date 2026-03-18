/*
 * Robot-B — XIAO ESP32-S3 PLUS
 * BLE RSSI distance (median + Kalman), NimBLE central
 * Kevin Walker 18 MAR 2026
 *
 * Auto-calibrates RSSI at 10 cm on connect.
 * Filtering: 16-sample median → log-distance model → 1D Kalman.
 * Close range clamped to 2 cm minimum.
 * Works with "Robot-A" running xiao_emitter or diag_emitter.
 *
 * Status LED (GPIO 21):
 *   blinking = scanning
 *   solid    = connected + calibrated
 *   off      = connected, waiting for cal
 */

#include <NimBLEDevice.h>
#include <math.h>

#define SERVICE_UUID  "19b10000-e8f2-537e-4f6c-d104768a1214"
#define TX_CHAR_UUID  "19b10001-e8f2-537e-4f6c-d104768a1214"
#define RX_CHAR_UUID  "19b10002-e8f2-537e-4f6c-d104768a1214"

#define LED_PIN 21
bool ledActiveHigh = false;  // XIAO ESP32-S3 Plus LED is typically active LOW
void ledOn()  { digitalWrite(LED_PIN, ledActiveHigh ? HIGH : LOW); }
void ledOff() { digitalWrite(LED_PIN, ledActiveHigh ? LOW : HIGH); }

const float CAL_DIST_CM = 10.0;  // actual distance during calibration
const float CLOSE_CM   = 2.0;   // minimum clamp for output

// ── RSSI median filter ──────────────────────────────────────

float RSSI_REF   = -50;
float PATH_LOSS  = 2.5;

const int MED_N  = 8;
float rssiBuf[16];
int   rssiBufIdx  = 0;
bool  rssiBufFull = false;
bool  rssiCal     = false;

void resetRssiBuf() {
  rssiBufIdx  = 0;
  rssiBufFull = false;
}

void addRssi(float r) {
  rssiBuf[rssiBufIdx++] = r;
  if (rssiBufIdx >= MED_N) { rssiBufIdx = 0; rssiBufFull = true; }
}

float medianRssi() {
  int n = rssiBufFull ? MED_N : rssiBufIdx;
  if (n == 0) return -999;
  float sorted[16];
  for (int i = 0; i < n; i++) sorted[i] = rssiBuf[i];
  for (int i = 1; i < n; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  if (n % 2 == 0) return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
  return sorted[n / 2];
}

// ── 1D Kalman filter on distance ────────────────────────────

float kX   = 0;
float kP   = 1000;
float kQ   = 2.0;
float kR   = 100.0;
bool  kInit = false;

void resetKalman() {
  kX = 0; kP = 1000; kInit = false;
}

float kalmanUpdate(float z) {
  if (!kInit) { kX = z; kP = kR; kInit = true; return kX; }
  kP += kQ;
  float K = kP / (kP + kR);
  kX += K * (z - kX);
  kP *= (1.0 - K);
  return kX;
}

// ── BLE state ───────────────────────────────────────────────

NimBLEClient*               pClient   = nullptr;
NimBLERemoteCharacteristic* pRemoteTx = nullptr;
NimBLERemoteCharacteristic* pRemoteRx = nullptr;
bool bleConnected = false;
float smoothDist  = -1;
float emitterHeading = 0;
unsigned long lastPrint = 0;
unsigned long lastSend  = 0;
unsigned long lastRssi  = 0;

// ── NimBLE callbacks ────────────────────────────────────────

class ClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pC) override {
    Serial.println("Connected");
  }

  void onDisconnect(NimBLEClient* pC, int reason) override {
    bleConnected = false;
    rssiCal      = false;
    resetRssiBuf();
    resetKalman();
    smoothDist   = -1;
    pRemoteTx    = nullptr;
    pRemoteRx    = nullptr;
    Serial.printf("Disconnected (reason=%d)\n", reason);
  }
};

static ClientCB clientCB;

// Notification handler for TX characteristic — parse emitter heading
void txNotifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t len, bool isNotify) {
  if (len >= 4 && pData[0] == 0xAA) {
    int16_t raw;
    memcpy(&raw, pData + 1, 2);
    emitterHeading = raw / 10.0;
  }
}

// ── Scan and connect ────────────────────────────────────────

bool connectToRobotA() {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(45);
  pScan->setWindow(45);
  pScan->clearResults();

  Serial.println("Scanning...");
  NimBLEScanResults results = pScan->getResults(3000);
  Serial.printf("Scan done, %d device(s)\n", results.getCount());

  const NimBLEAdvertisedDevice* target = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    std::string name = dev->getName();
    if (name.length() > 0) {
      Serial.printf("  [%d] %s  name='%s'  RSSI=%d  svcUUID=%s\n",
        i,
        dev->getAddress().toString().c_str(),
        name.c_str(),
        dev->getRSSI(),
        dev->haveServiceUUID() ? dev->getServiceUUID().toString().c_str() : "none");
    }
    if (name == "Robot-A") {
      target = dev;
    }
  }

  if (!target) {
    Serial.println("Robot-A not found");
    delay(200);
    return false;
  }

  Serial.printf("Found Robot-A: %s  RSSI=%d\n",
    target->getAddress().toString().c_str(),
    target->getRSSI());

  if (!pClient) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCB, false);
    pClient->setConnectionParams(24, 48, 0, 400);
    pClient->setConnectTimeout(10);
  }

  if (!pClient->connect(target)) {
    Serial.println("Connect failed");
    delay(200);
    return false;
  }

  Serial.printf("Connected, RSSI=%d\n", pClient->getRssi());

  NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
  if (!pSvc) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }

  pRemoteTx = pSvc->getCharacteristic(TX_CHAR_UUID);
  pRemoteRx = pSvc->getCharacteristic(RX_CHAR_UUID);
  if (!pRemoteTx || !pRemoteRx) {
    Serial.println("Characteristics not found");
    pClient->disconnect();
    return false;
  }

  if (pRemoteTx->canNotify()) {
    pRemoteTx->subscribe(true, txNotifyCB);
  }

  bleConnected = true;
  return true;
}

// ── Setup ───────────────────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOn();

  Serial.begin(115200);
  delay(500);

  NimBLEDevice::init("Robot-B");
  NimBLEDevice::setPower(9);

  Serial.println("Robot-B receiver ready");
}

// ── Loop ────────────────────────────────────────────────────

void loop() {
  if (!bleConnected) {
    (millis() / 500) % 2 ? ledOn() : ledOff();
    static int scanFails = 0;
    if (!connectToRobotA()) {
      scanFails++;
      if (scanFails % 5 == 0) {
        Serial.printf("(%d scans without connection)\n", scanFails);
      }
      return;
    }
    Serial.printf("Connected after %d scan attempt(s)\n", scanFails + 1);
    scanFails = 0;
  }

  if (!pClient->isConnected()) {
    bleConnected = false;
    rssiCal      = false;
    resetRssiBuf();
    resetKalman();
    smoothDist   = -1;
    Serial.println("Lost connection");
    return;
  }

  // ── RSSI (rate-limited to every 100 ms) ──
  if (millis() - lastRssi >= 100) {
    lastRssi = millis();
    int rssiVal = pClient->getRssi();
    if (rssiVal != 0 && rssiVal != 127) {
      addRssi(rssiVal);
      float med = medianRssi();

      if (!rssiCal && rssiBufFull) {
        RSSI_REF = med + 10.0 * PATH_LOSS * log10(CAL_DIST_CM / 100.0);
        rssiCal  = true;
        Serial.printf("Calibrated: medRSSI=%.1f  ref1m=%.1f  calDist=%dcm\n",
          med, RSSI_REF, (int)CAL_DIST_CM);
      }

      if (rssiCal) {
        float rawDist = pow(10.0, (RSSI_REF - med) / (10.0 * PATH_LOSS)) * 100.0;
        if (rawDist < CLOSE_CM) rawDist = CLOSE_CM;
        if (rawDist > 500) rawDist = 500;
        smoothDist = kalmanUpdate(rawDist);
      }
    }
  }

  // ── Send distance to Robot-A every 500 ms ──
  if (pRemoteRx && millis() - lastSend >= 500) {
    lastSend = millis();
    uint8_t pkt[6];
    pkt[0] = 0xBB;
    int16_t dummy = 0;
    memcpy(pkt + 1, &dummy, 2);
    int16_t d = (rssiCal && smoothDist >= 0) ? (int16_t)smoothDist : (int16_t)-1;
    memcpy(pkt + 3, &d, 2);
    pkt[5] = 0;
    pRemoteRx->writeValue(pkt, 6, false);
  }

  // ── LED: solid when connected ──
  ledOn();

  // ── Serial ──
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    if (smoothDist >= 0) {
      Serial.printf("%dcm  heading=%.1f°\n", (int)smoothDist, emitterHeading);
    } else {
      Serial.printf("--  heading=%.1f°\n", emitterHeading);
    }
  }

  delay(20);
}
