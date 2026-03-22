/*
 * Audio Test — Arduino Nano 33 BLE Sense Rev 2 + MAX98357A
 * Kevin Walker
 * 
 * Plays each .mp3 file in sequence on boot, then use keys to play.
 *
 * Wiring:
 *   Arduino D0 → DFPlayer RX
 *   Arduino D1 → DFPlayer TX
 *   Arduino 3V3 → DFPlayer VCC
 *   Arduino GND → DFPlayer GND
 *   Speaker → DFPlayer SPK terminals
 *
 */

#include <DFRobotDFPlayerMini.h>

DFRobotDFPlayerMini myDFPlayer;

void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("=================================");
  Serial.println("    DFPlayer Mini Test");
  Serial.println("=================================");
  Serial.println();
  
  // Hardware Serial1 for DFPlayer (D0/D1)
  Serial1.begin(9600);
  delay(1000);  // Give DFPlayer time to initialize
  
  Serial.println("Initializing DFPlayer...");
  Serial.println("(This may take a few seconds)");
  
  if (!myDFPlayer.begin(Serial1)) {
    Serial.println();
    Serial.println("❌ DFPlayer initialization FAILED!");
    Serial.println();
    Serial.println("Troubleshooting checklist:");
    Serial.println("1. Is SD card inserted?");
    Serial.println("2. Is SD card formatted FAT32?");
    Serial.println("3. Are MP3 files named correctly? (0001.mp3, 0002.mp3...)");
    Serial.println("4. Is DFPlayer powered? (Check 3.3V on VCC pin)");
    Serial.println("5. Are RX/TX connections correct?");
    Serial.println("   - Arduino D1 → 1kΩ → DFPlayer RX");
    Serial.println("   - Arduino D0 → DFPlayer TX");
    Serial.println("6. Is speaker connected to SPK outputs?");
    Serial.println();
    
    while(true) {
      delay(1000);  // Stop here
    }
  }
  
  Serial.println("✅ DFPlayer initialized successfully!");
  Serial.println();
  
  // Configure DFPlayer
  myDFPlayer.volume(30);  // Volume: 0-30 (start at 20)
  delay(100);
  
  // Get info about SD card
  int fileCount = myDFPlayer.readFileCounts();
  Serial.print("Files found on SD card: ");
  Serial.println(fileCount);
  
  if (fileCount == 0) {
    Serial.println();
    Serial.println("⚠️  WARNING: No files found!");
    Serial.println("Check SD card has MP3 files named:");
    Serial.println("0001.mp3, 0002.mp3, etc.");
  }
  
  Serial.println();
  Serial.println("=================================");
  Serial.println("Starting playback test...");
  Serial.println("=================================");
  Serial.println();
  delay(2000);
}

void loop() {
  // Play track 1
  Serial.println("▶ Playing track 0001.mp3");
  myDFPlayer.play(1);
  delay(5000);  // Play for 5 seconds
  
  Serial.println("⏸ Pause");
  delay(2000);
  
  // Play track 2
  Serial.println("▶ Playing track 0002.mp3");
  myDFPlayer.play(2);
  delay(5000);
  
  Serial.println("⏸ Pause");
  delay(2000);
  
  // Play track 3
  Serial.println("▶ Playing track 0003.mp3");
  myDFPlayer.play(3);
  delay(5000);
  
  Serial.println();
  Serial.println("--- Test cycle complete ---");
  Serial.println("Repeating in 3 seconds...");
  Serial.println();
  delay(3000);
}
