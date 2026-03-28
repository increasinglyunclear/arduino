/*
 * MaUWB ESP32S3 — Tag 0
 * Kevin Walker 27 Mar 2026
 * 
 * Configures the UWB module as Tag #0 via AT commands.
 * Parses AT+RANGE responses to extract distance to Anchor 0.
 * Shows live distance on the onboard SSD1306 OLED.
 *
 * Board: ESP32S3 Dev Module
 * USB-C for power + serial (115200 baud)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SERIAL_LOG Serial
HardwareSerial SERIAL_AT(2);

#define UWB_INDEX  0
#define RESET_PIN  16
#define IO_RXD2    18
#define IO_TXD2    17
#define I2C_SDA    39
#define I2C_SCL    38

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

String response = "";
float lastDist = -1;
unsigned long lastUpdate = 0;

String sendAT(const String& cmd, unsigned long timeout) {
  SERIAL_LOG.println(cmd);
  SERIAL_AT.println(cmd);

  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (SERIAL_AT.available()) {
      resp += (char)SERIAL_AT.read();
    }
  }
  SERIAL_LOG.println(resp);
  return resp;
}

void oledShow(const String& line1, const String& line2 = "") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length()) {
    display.setCursor(0, 20);
    display.setTextSize(2);
    display.println(line2);
  }
  display.display();
}

void parseRange(const String& line) {
  // Format: AT+RANGE=tid:0,mask:01,seq:N,range:(d0,d1,...,d7),ancid:(a0,a1,...,a7)
  int rangeStart = line.indexOf("range:(");
  int ancidStart = line.indexOf("ancid:(");
  if (rangeStart < 0 || ancidStart < 0) return;

  rangeStart += 7; // skip "range:("
  int rangeEnd = line.indexOf(')', rangeStart);
  ancidStart += 7; // skip "ancid:("
  int ancidEnd = line.indexOf(')', ancidStart);
  if (rangeEnd < 0 || ancidEnd < 0) return;

  String rangeStr = line.substring(rangeStart, rangeEnd);
  String ancidStr = line.substring(ancidStart, ancidEnd);

  int ranges[8] = {0};
  int ancids[8] = {-1};

  int idx = 0;
  int pos = 0;
  while (idx < 8 && pos <= (int)rangeStr.length()) {
    int comma = rangeStr.indexOf(',', pos);
    if (comma < 0) comma = rangeStr.length();
    ranges[idx++] = rangeStr.substring(pos, comma).toInt();
    pos = comma + 1;
  }

  idx = 0; pos = 0;
  while (idx < 8 && pos <= (int)ancidStr.length()) {
    int comma = ancidStr.indexOf(',', pos);
    if (comma < 0) comma = ancidStr.length();
    ancids[idx++] = ancidStr.substring(pos, comma).toInt();
    pos = comma + 1;
  }

  for (int i = 0; i < 8; i++) {
    if (ancids[i] < 0) continue;
    SERIAL_LOG.printf("  → Anchor %d: %d cm\n", ancids[i], ranges[i]);

    lastDist = ranges[i];
    lastUpdate = millis();

    String top = "Tag 0 -> Anchor " + String(ancids[i]);
    String big = String(ranges[i]) + " cm";
    oledShow(top, big);
  }
}

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  SERIAL_LOG.begin(115200);
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOK = true;
    display.clearDisplay();
  }

  oledShow("MaUWB Tag 0", "Init...");
  SERIAL_LOG.println("=== MaUWB Tag 0 ===");

  sendAT("AT?", 2000);
  sendAT("AT+RESTORE", 5000);

  // AT+SETCFG=id,role,speed,filter
  //   role: 0=tag, speed: 1=6.8M, filter: 1=on
  String cfg = "AT+SETCFG=" + String(UWB_INDEX) + ",0,1,1";
  sendAT(cfg, 2000);

  // AT+SETCAP=tagCount,slotMs,extMode
  sendAT("AT+SETCAP=64,10,1", 2000);
  sendAT("AT+SETRPT=1", 2000);
  sendAT("AT+SAVE", 2000);
  sendAT("AT+RESTART", 2000);

  oledShow("MaUWB Tag 0", "Ready");
  SERIAL_LOG.println("Tag 0 ready — looking for anchors");
}

void loop() {
  // Forward serial → AT (for manual AT commands)
  while (SERIAL_LOG.available()) {
    SERIAL_AT.write(SERIAL_LOG.read());
  }

  // Read AT responses line by line
  while (SERIAL_AT.available()) {
    char c = SERIAL_AT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (response.length() > 0) {
        SERIAL_LOG.println(response);
        if (response.startsWith("AT+RANGE")) {
          parseRange(response);
        }
        response = "";
      }
    } else {
      response += c;
    }
  }

  // Show "no signal" if no update for 3s
  if (lastDist >= 0 && millis() - lastUpdate > 3000) {
    lastDist = -1;
    oledShow("Tag 0", "No signal");
  }
}
