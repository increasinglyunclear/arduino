/*
 * MaUWB ESP32S3 — Anchor 0
 * Kevin Walker 27 Mar 2026
 * 
 * Configures the UWB module as Anchor #0 via AT commands.
 * Prints all UWB responses to Serial for debugging.
 * Shows status on the onboard SSD1306 OLED.
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

void oledStatus(const String& line1, const String& line2 = "") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length()) {
    display.setCursor(0, 16);
    display.setTextSize(2);
    display.println(line2);
  }
  display.display();
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

  oledStatus("MaUWB Anchor 0", "Init...");
  SERIAL_LOG.println("=== MaUWB Anchor 0 ===");

  sendAT("AT?", 2000);
  sendAT("AT+RESTORE", 5000);

  // AT+SETCFG=id,role,speed,filter
  //   role: 1=anchor, speed: 1=6.8M, filter: 1=on
  String cfg = "AT+SETCFG=" + String(UWB_INDEX) + ",1,1,1";
  sendAT(cfg, 2000);

  // AT+SETCAP=tagCount,slotMs,extMode
  sendAT("AT+SETCAP=64,10,1", 2000);
  sendAT("AT+SETRPT=1", 2000);
  sendAT("AT+SAVE", 2000);
  sendAT("AT+RESTART", 2000);

  oledStatus("MaUWB Anchor 0", "Ready");
  SERIAL_LOG.println("Anchor 0 ready — waiting for tags");
}

void loop() {
  // Forward serial → AT (for manual AT commands)
  while (SERIAL_LOG.available()) {
    SERIAL_AT.write(SERIAL_LOG.read());
  }

  // Print AT responses
  while (SERIAL_AT.available()) {
    char c = SERIAL_AT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      SERIAL_LOG.println();
    } else {
      SERIAL_LOG.print(c);
    }
  }
}
