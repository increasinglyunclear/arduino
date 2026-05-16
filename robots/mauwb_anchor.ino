/*
 * MaUWB Anchor Monitor + BLE Command Relay
 *
 * Powers the MaUWB chipset (pre-configured as anchor via ST-Link).
 * Scans BLE for robot position broadcasts → relays to USB serial.
 * Receives commands from the web page via USB serial → broadcasts
 * them to robots via BLE advertising ("AnchorCmd").
 *
 * Serial command protocol (from web page):
 *   >TT:HH:HH:...\n   TT=target hex (00/01/FF), HH=cmd bytes
 *
 * Wiring (XIAO ESP32S3-PLUS → MaUWB DW3000 Chipset):
 *   3V3 → VCC,  GND → GND
 *   D7 (GPIO44) → Chipset UART1TX  (optional monitoring)
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define UWB_RX_PIN 44
#define LED_PIN    21

BLEScan* pScan = nullptr;
BLEAdvertising* pAdv = nullptr;

unsigned long lastBlink = 0;
bool ledState = false;

// Command relay
uint8_t cmdSeq = 0;
uint8_t cmdTarget = 0xFF;
uint8_t cmdPayload[12];
uint8_t cmdPayloadLen = 0;
unsigned long cmdSentAt = 0;
const unsigned long CMD_BROADCAST_MS = 5000;
bool cmdActive = false;
String serialBuf;

// Scan diagnostics — counters published every ~3 s so the web page sees scanner health.
volatile uint32_t scanCnt = 0;       // total advertised devices seen
volatile uint32_t scanNamed = 0;     // advertisements carrying a name
volatile uint32_t scanWithMf = 0;    // advertisements carrying any manufacturer data
volatile uint32_t scanFFFF = 0;      // manuf data starting with 0xFF 0xFF (our robots)
volatile uint32_t scanRobot = 0;     // name match (RobotA/B)
volatile uint32_t scanRobotShort = 0;
unsigned long lastFFFFDumpMs = 0;

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    scanCnt++;
    bool named = dev.haveName();
    if (named) scanNamed++;
    String name = named ? String(dev.getName().c_str()) : String("");

    // Identify our robots by manufacturer-data 0xFF 0xFF prefix — bypasses any name-parsing issue.
    String mfRaw;
    const uint8_t* mf = nullptr;
    int len = 0;
    if (dev.haveManufacturerData()) {
      scanWithMf++;
      mfRaw = dev.getManufacturerData();
      mf = (const uint8_t*)mfRaw.c_str();
      len = mfRaw.length();
    }
    bool isFFFF = (mf && len >= 2 && mf[0] == 0xFF && mf[1] == 0xFF);
    if (isFFFF) {
      scanFFFF++;
      // Dump bytes of the first few matches so we can see payload format.
      if (millis() - lastFFFFDumpMs > 2000) {
        lastFFFFDumpMs = millis();
        Serial.printf("MF:len=%d,name='%s',rssi=%d,bytes=", len, name.c_str(), dev.getRSSI());
        for (int i = 0; i < len && i < 16; i++) Serial.printf("%02X ", mf[i]);
        Serial.println();
      }
    }

    // Tag ID is encoded in bit 7 of the status byte in the primary ADV
    // manufacturer data. This avoids the old bug where two simultaneously-
    // advertising robots could have their ADV+SCAN_RSP data cross-barred by
    // the BLE stack, causing T1's distances to be reported as if they came
    // from T0. If the payload looks wrong we fall back to the BLE name.
    if (!isFFFF || len < 8) {
      if (name == "RobotA-Tx" || name == "RobotB-Rx") scanRobotShort++;
      return;
    }
    int off = 2;
    uint16_t d0     = ((uint16_t)mf[off    ] << 8) | mf[off + 1];
    uint16_t d1     = ((uint16_t)mf[off + 2] << 8) | mf[off + 3];
    uint8_t  status =  mf[off + 4];
    uint8_t  hdg    =  mf[off + 5];
    int tid = (status >> 7) & 1;   // authoritative: from ADV payload

    // Optional heap field — bytes 12..13 (offsets 10..11 from `off`),
    // present only on robots running firmware with the 14-byte payload.
    // 0 means "not reported" (older firmware or not yet measured).
    uint32_t heapBytes = 0;
    if (len - off >= 12) {
      uint16_t heapUnits = ((uint16_t)mf[off + 10] << 8) | mf[off + 11];
      heapBytes = (uint32_t)heapUnits << 5;   // *32
    }

    // Sanity: if we also saw a name, warn (don't override) if it disagrees.
    // This catches a stale-firmware robot with the old payload (bit 7 = 0).
    if (name.length() && !(name == "RobotA-Tx" || name == "RobotB-Rx")) {
      /* foreign device that happens to have 0xFF 0xFF prefix — ignore */
      return;
    }
    scanRobot++;

    Serial.printf("TAG:%d,d0:%d,d1:%d,st:%d,hdg:%d,rssi:%d,heap:%u\n",
                  tid, d0, d1, status, hdg * 2, dev.getRSSI(),
                  (unsigned)heapBytes);
  }
};

void updateCmdAd() {
  BLEAdvertisementData ad;
  ad.setName("AnchorCmd");
  if (cmdActive) {
    uint8_t mf[16] = { 0xFF, 0xFF, cmdSeq, cmdTarget, cmdPayloadLen };
    memcpy(mf + 5, cmdPayload, cmdPayloadLen);
    ad.setManufacturerData(String((char*)mf, 5 + cmdPayloadLen));
  }
  pAdv->setAdvertisementData(ad);
}

void handleSerialCmd(const String& line) {
  if (line.length() < 4) return;
  uint8_t parts[16];
  int n = 0, p = 0;
  while (p < (int)line.length() && n < 16) {
    int c = line.indexOf(':', p);
    if (c < 0) c = line.length();
    parts[n++] = (uint8_t)strtol(line.substring(p, c).c_str(), NULL, 16);
    p = c + 1;
  }
  if (n < 2) return;
  cmdTarget = parts[0];
  cmdPayloadLen = n - 1;
  memcpy(cmdPayload, parts + 1, cmdPayloadLen);
  cmdSeq++;
  cmdSentAt = millis();
  cmdActive = true;
  // Rotate the advertisement payload IN PLACE. Previously we did
  // pAdv->stop(); updateCmdAd(); pAdv->start();
  // which creates a 30-50 ms window where nothing is on air — during rapid
  // command sequences (e.g. one cmd per second from the test-sequence UI)
  // the robot's BLE scanner could land in that gap and miss a command.
  // setAdvertisementData() replaces the payload without stopping the radio,
  // so the new cmdSeq/payload is visible to the next scan window with no dark
  // time. Robots dedupe by cmdSeq so they won't execute the same cmd twice.
  updateCmdAd();
  Serial.printf("CMD>> seq=%d tgt=0x%02X len=%d\n", cmdSeq, cmdTarget, cmdPayloadLen);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial1.begin(115200, SERIAL_8N1, UWB_RX_PIN, -1);
  delay(500);

  Serial.println("\n=== MaUWB Anchor + BLE Relay + Cmd ===");

  BLEDevice::init("AnchorRelay");

  pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  pScan->setActiveScan(true);   // send SCAN_REQ so we receive SCAN_RSP with the name
  pScan->setInterval(100);
  pScan->setWindow(80);
  pScan->start(0, nullptr, false);

  pAdv = BLEDevice::getAdvertising();
  // Force a tight, steady advertising interval (20 ms between packets).
  // Default is ~1280 ms, which means commands sit on air but only one
  // packet per second actually gets transmitted — if the robot's scan
  // window doesn't overlap with that one packet, the command is lost.
  // BLE spec min is 20 ms (32 * 0.625 ms). Setting min == max disables
  // the stack's jitter so packets land on a predictable 20 ms grid.
  pAdv->setMinInterval(0x20);  // 32 * 0.625 ms = 20 ms
  pAdv->setMaxInterval(0x20);
  updateCmdAd();
  pAdv->start();

  digitalWrite(LED_PIN, LOW);
  Serial.println("Ready. Scanning + command relay active.\n");
}

void loop() {
  while (Serial1.available()) Serial.write(Serial1.read());

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.startsWith(">")) handleSerialCmd(serialBuf.substring(1));
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }

  if (cmdActive && millis() - cmdSentAt >= CMD_BROADCAST_MS) {
    cmdActive = false;
    // Same rationale as in handleSerialCmd(): update the payload in place
    // rather than bouncing the advertiser. This emits the "idle" (no cmd)
    // payload without a dark window.
    updateCmdAd();
  }

  static unsigned long lastRestart = 0;
  if (millis() - lastRestart >= 30000) {
    lastRestart = millis();
    pScan->stop();
    delay(50);
    pScan->start(0, nullptr, false);
  }

  if (millis() - lastBlink >= 2000) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH);
  }

  static unsigned long lastStat = 0;
  if (millis() - lastStat >= 3000) {
    lastStat = millis();
    Serial.printf("SCAN:all=%lu,named=%lu,withMf=%lu,ffff=%lu,robot=%lu,short=%lu\n",
                  (unsigned long)scanCnt, (unsigned long)scanNamed,
                  (unsigned long)scanWithMf, (unsigned long)scanFFFF,
                  (unsigned long)scanRobot, (unsigned long)scanRobotShort);
  }
}
