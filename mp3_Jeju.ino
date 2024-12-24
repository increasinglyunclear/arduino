// Jeju MP3 player
// Kevin Walker 20 Nov 2022
// for playing MP3 files in live event
// commissioned by British Council for Daily Rituals project
// https://nine-earths.net/dailyrituals/

#include <SFEMP3ShieldConfig.h>
#include <SFEMP3Shield.h>
#include <SFEMP3Shieldmainpage.h>
#include <SdFatConfig.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <BlockDriver.h>
#include <SysCall.h>
#include <SPI.h>
#include <SdFat.h>
#include <FreeStack.h>
#include <SFEMP3Shield.h>

SdFat sd;
SFEMP3Shield MP3player;
//int16_t last_ms_char; // milliseconds of last recieved character from Serial port.
//int8_t buffer_pos; // next position to recieve character from Serial port.

//char buffer[6]; // 0-35K+null

void setup() {

  //uint8_t result; //result code from some function as to be tested at later time.

  Serial.begin(115200);

  //Initialize the SdCard.
  if (!sd.begin(SD_SEL, SPI_FULL_SPEED)) sd.initErrorHalt();
  // depending upon your SdCard environment, SPI_HAVE_SPEED may work better.
  if (!sd.chdir("/")) sd.errorHalt("sd.chdir");

  //Initialize the MP3 Player Shield
  MP3player.begin();
}

void loop() {

  if (!MP3player.isPlaying()) {

    String trackname;
    int x = random(28 + 1);
    if (x < 10) {
      trackname = "jeju000" + String(x) + ".mp3";
    } else {
      //if (x < 100) {
      trackname = "jeju00 " + String(x) + ".mp3";
      //      } else {
      //        trackname = "track" + String(x) + ".mp3";
    }
    // }
    char filename[13];
    trackname.toCharArray(filename, 13);
    Serial.println(filename);
    MP3player.playMP3(filename);
    //MP3player.playTrack(x);



    //MP3player.getAudioInfo();

    //int x = random(395 + 1);
    //Serial.println(x);

    //    SdFile file;
    //    char filename[13];
    //    char tracklist[3160];
    //    sd.chdir("/", true);
    //    uint16_t count = 1;
    //    while (file.openNext(sd.vwd(), O_READ))
    //    {
    //      file.getName(filename, sizeof(filename));
    //      if ( isFnMusic(filename) ) {
    //        if (count = x) {
    //          Serial.println(count);
    //          //MP3player.playMP3(filename);
    //          break;
    //        }
    //        else {
    //          //strcat( tracklist, filename );
    //          count++;
    //        }
    //      }
    //      file.close();
    //    }
    //    //Serial.println(tracklist);
    //    MP3player.playMP3(filename);


    //    switch (x) { // play some random Jeju sounds
    //      case 1:
    //        MP3player.playMP3("track073.mp3");
    //        break;
    //      case 2:
    //        MP3player.playMP3("track074.mp3");
    //        break;
    //      case 3:
    //        MP3player.playMP3("track075.mp3");
    //        break;
    //    }
  }
}
