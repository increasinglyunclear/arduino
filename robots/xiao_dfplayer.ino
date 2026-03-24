/*
 * DFPlayer Mini Test — XIAO ESP32-S3 Plus
 * Kevin Walker 23 Mar 2026
 *
 * Interactive test for DFPlayer Mini via Serial.
 * Plays MP3/WAV files from micro SD card.
 *
 * Wiring:
 *   D8  (GPIO7)  → 1K resistor → DFPlayer RX
 *   D9  (GPIO8)  → DFPlayer TX
 *   3V3          → DFPlayer VCC
 *   GND          → DFPlayer GND
 *   Speaker +/-  → DFPlayer SPK1/SPK2
 *
 * SD card (FAT32): files named 0001.mp3, 0002.mp3, etc. in root.
 *
 * Serial keys:
 *   1-9  = play track N
 *   n    = next track
 *   p    = pause / resume
 *   x    = stop
 *   +/-  = volume up/down
 *   ?    = show status
 */

#include <DFRobotDFPlayerMini.h>

#define DFPLAYER_TX  D8  // XIAO TX → DFPlayer RX (via 1K resistor)
#define DFPLAYER_RX  D9  // XIAO RX ← DFPlayer TX
#define LED_PIN      21

bool ledActiveHigh = false;
void ledOn()  { digitalWrite(LED_PIN, ledActiveHigh ? HIGH : LOW); }
void ledOff() { digitalWrite(LED_PIN, ledActiveHigh ? LOW : HIGH); }

DFRobotDFPlayerMini dfPlayer;
int currentVol = 25;
int totalFiles = 0;
bool paused = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOn();

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== XIAO DFPlayer Test ===");

  Serial1.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(500);

  Serial.println("Connecting to DFPlayer...");
  int retries = 0;
  while (!dfPlayer.begin(Serial1)) {
    retries++;
    Serial.printf("  attempt %d failed\n", retries);
    if (retries >= 5) {
      Serial.println("ERROR: DFPlayer not detected!");
      Serial.println("Check wiring: D8→1K→RX, D9←TX, 3V3→VCC, GND→GND");
      Serial.println("Check SD card is inserted and FAT32 formatted.");
      while (true) {
        ledOn(); delay(200);
        ledOff(); delay(200);
      }
    }
    delay(1000);
  }

  Serial.println("DFPlayer OK");

  dfPlayer.volume(currentVol);
  delay(100);

  totalFiles = dfPlayer.readFileCounts();
  delay(100);
  Serial.printf("SD card: %d file(s)\n", totalFiles);

  if (totalFiles <= 0) {
    Serial.println("WARNING: No files found on SD card.");
    Serial.println("Place files named 0001.mp3, 0002.mp3, etc. in root.");
  }

  Serial.println("\nKeys: 1-9=play track  n=next  p=pause/resume  x=stop");
  Serial.println("      +/-=volume     ?=status");
  Serial.printf("Volume: %d/30\n\n", currentVol);

  if (totalFiles > 0) {
    Serial.println("Playing track 1...");
    dfPlayer.play(1);
  }
}

void printStatus() {
  Serial.printf("Volume: %d/30  Files: %d  State: %d\n",
    currentVol, totalFiles, dfPlayer.readState());
}

void loop() {
  ledOn();

  if (dfPlayer.available()) {
    uint8_t type = dfPlayer.readType();
    int value = dfPlayer.read();
    switch (type) {
      case DFPlayerPlayFinished:
        Serial.printf("Track %d finished\n", value);
        break;
      case DFPlayerCardInserted:
        Serial.println("SD card inserted");
        break;
      case DFPlayerCardRemoved:
        Serial.println("SD card removed");
        break;
      case DFPlayerError:
        Serial.printf("DFPlayer error: %d\n", value);
        break;
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': case '2': case '3': case '4': case '5':
      case '6': case '7': case '8': case '9': {
        int track = c - '0';
        Serial.printf("Playing track %d\n", track);
        dfPlayer.play(track);
        paused = false;
        break;
      }
      case 'n':
        Serial.println("Next track");
        dfPlayer.next();
        paused = false;
        break;
      case 'p':
        if (paused) {
          Serial.println("Resume");
          dfPlayer.start();
          paused = false;
        } else {
          Serial.println("Paused");
          dfPlayer.pause();
          paused = true;
        }
        break;
      case 'x':
        Serial.println("Stop");
        dfPlayer.stop();
        paused = false;
        break;
      case '+': case '=':
        currentVol = min(30, currentVol + 2);
        dfPlayer.volume(currentVol);
        Serial.printf("Volume: %d/30\n", currentVol);
        break;
      case '-':
        currentVol = max(0, currentVol - 2);
        dfPlayer.volume(currentVol);
        Serial.printf("Volume: %d/30\n", currentVol);
        break;
      case '?':
        printStatus();
        break;
    }
  }

  delay(20);
}
