// Final code for 'F0lded 1n' installation https://increasinglyunclear.world/folded/
// Plays random files from MP3 shield
// Kevin Walker Feb 2023

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

SdFat sd;
SFEMP3Shield MP3player;

void setup() {
  //Initialize the SdCard & player
  if (!sd.begin(SD_SEL, SPI_FULL_SPEED)) sd.initErrorHalt();
  if (!sd.chdir("/")) sd.errorHalt("sd.chdir");
  MP3player.begin();

  //MP3player.playTrack(00010);
  //MP3player.playMP3(trackName);
}

void loop() {
  if (!MP3player.isPlaying()) {
    int x = random(378 + 1);
    //MP3player.playTrack(x);

    // KLUDGE: WORKAROUND BECAUSE FILENAME BUILDING WASN'T WORKING
    
    switch (x) {
      case 1:
        MP3player.playMP3("track300.mp3");
        break;
      case 2:
        MP3player.playMP3("track302.mp3");
        break;
      case 3:
        MP3player.playMP3("track303.mp3");
        break;
      case 4:
        MP3player.playMP3("track304.mp3");
        break;
      case 5:
        MP3player.playMP3("track305.mp3");
        break;
      case 6:
        MP3player.playMP3("track306.mp3");
        break;
      case 7:
        MP3player.playMP3("track307.mp3");
        break;
      case 8:
        MP3player.playMP3("track308.mp3");
        break;
      case 9:
        MP3player.playMP3("track309.mp3");
        break;
      case 10:
        MP3player.playMP3("track310.mp3");
        break;
      case 11:
        MP3player.playMP3("track300.mp3");
        break;
      case 12:
        MP3player.playMP3("track302.mp3");
        break;
      case 13:
        MP3player.playMP3("track303.mp3");
        break;
      case 14:
        MP3player.playMP3("track304.mp3");
        break;
      case 15:
        MP3player.playMP3("track305.mp3");
        break;
      case 16:
        MP3player.playMP3("track306.mp3");
        break;
      case 17:
        MP3player.playMP3("track307.mp3");
        break;
      case 18:
        MP3player.playMP3("track308.mp3");
        break;
      case 19:
        MP3player.playMP3("track309.mp3");
        break;
      case 20:
        MP3player.playMP3("track310.mp3");
        break;
      case 21:
        MP3player.playMP3("track300.mp3");
        break;
      case 22:
        MP3player.playMP3("track302.mp3");
        break;
      case 23:
        MP3player.playMP3("track303.mp3");
        break;
      case 24:
        MP3player.playMP3("track304.mp3");
        break;
      case 25:
        MP3player.playMP3("track305.mp3");
        break;
      case 26:
        MP3player.playMP3("track306.mp3");
        break;
      case 27:
        MP3player.playMP3("track307.mp3");
        break;
      case 28:
        MP3player.playMP3("track308.mp3");
        break;
      case 29:
        MP3player.playMP3("track309.mp3");
        break;
      case 30:
        MP3player.playMP3("track310.mp3");
        break;
      case 31:
        MP3player.playMP3("track300.mp3");
        break;
      case 32:
        MP3player.playMP3("track302.mp3");
        break;
      case 33:
        MP3player.playMP3("track303.mp3");
        break;
      case 34:
        MP3player.playMP3("track304.mp3");
        break;
      case 35:
        MP3player.playMP3("track305.mp3");
        break;
      case 36:
        MP3player.playMP3("track306.mp3");
        break;
      case 37:
        MP3player.playMP3("track307.mp3");
        break;
      case 38:
        MP3player.playMP3("track308.mp3");
        break;
      case 39:
        MP3player.playMP3("track309.mp3");
        break;
      case 40:
        MP3player.playMP3("track310.mp3");
        break;
      case 41:
        MP3player.playMP3("track300.mp3");
        break;
      case 42:
        MP3player.playMP3("track302.mp3");
        break;
      case 43:
        MP3player.playMP3("track303.mp3");
        break;
      case 44:
        MP3player.playMP3("track304.mp3");
        break;
      case 45:
        MP3player.playMP3("track305.mp3");
        break;
      case 46:
        MP3player.playMP3("track306.mp3");
        break;
      case 47:
        MP3player.playMP3("track307.mp3");
        break;
      case 48:
        MP3player.playMP3("track308.mp3");
        break;
      case 49:
        MP3player.playMP3("track309.mp3");
        break;
      case 50:
        MP3player.playMP3("track310.mp3");
        break;
      case 51:
        MP3player.playMP3("track300.mp3");
        break;
      case 52:
        MP3player.playMP3("track302.mp3");
        break;
      case 53:
        MP3player.playMP3("track303.mp3");
        break;
      case 54:
        MP3player.playMP3("track304.mp3");
        break;
      case 55:
        MP3player.playMP3("track305.mp3");
        break;
      case 56:
        MP3player.playMP3("track306.mp3");
        break;
      case 57:
        MP3player.playMP3("track307.mp3");
        break;
      case 58:
        MP3player.playMP3("track308.mp3");
        break;
      case 59:
        MP3player.playMP3("track309.mp3");
        break;
      case 60:
        MP3player.playMP3("track310.mp3");
        break;
      case 61:
        MP3player.playMP3("track300.mp3");
        break;
      case 62:
        MP3player.playMP3("track302.mp3");
        break;
      case 63:
        MP3player.playMP3("track303.mp3");
        break;
      case 64:
        MP3player.playMP3("track304.mp3");
        break;
      case 65:
        MP3player.playMP3("track305.mp3");
        break;
      case 66:
        MP3player.playMP3("track306.mp3");
        break;
      case 67:
        MP3player.playMP3("track307.mp3");
        break;
      case 68:
        MP3player.playMP3("track308.mp3");
        break;
      case 69:
        MP3player.playMP3("track309.mp3");
        break;
      case 70:
        MP3player.playMP3("track310.mp3");
        break;
      case 71:
        MP3player.playMP3("track300.mp3");
        break;
      case 72:
        MP3player.playMP3("track302.mp3");
        break;
      case 73:
        MP3player.playMP3("track303.mp3");
        break;
      case 74:
        MP3player.playMP3("track304.mp3");
        break;
      case 75:
        MP3player.playMP3("track305.mp3");
        break;
      case 76:
        MP3player.playMP3("track306.mp3");
        break;
      case 77:
        MP3player.playMP3("track307.mp3");
        break;
      case 78:
        MP3player.playMP3("track308.mp3");
        break;
      case 79:
        MP3player.playMP3("track309.mp3");
        break;
      case 80:
        MP3player.playMP3("track310.mp3");
        break;
      case 81:
        MP3player.playMP3("track300.mp3");
        break;
      case 82:
        MP3player.playMP3("track302.mp3");
        break;
      case 83:
        MP3player.playMP3("track303.mp3");
        break;
      case 84:
        MP3player.playMP3("track304.mp3");
        break;
      case 85:
        MP3player.playMP3("track305.mp3");
        break;
      case 86:
        MP3player.playMP3("track306.mp3");
        break;
      case 87:
        MP3player.playMP3("track307.mp3");
        break;
      case 88:
        MP3player.playMP3("track308.mp3");
        break;
      case 89:
        MP3player.playMP3("track309.mp3");
        break;
      case 90:
        MP3player.playMP3("track310.mp3");
        break;
      case 91:
        MP3player.playMP3("track300.mp3");
        break;
      case 92:
        MP3player.playMP3("track302.mp3");
        break;
      case 93:
        MP3player.playMP3("track303.mp3");
        break;
      case 94:
        MP3player.playMP3("track304.mp3");
        break;
      case 95:
        MP3player.playMP3("track305.mp3");
        break;
      case 96:
        MP3player.playMP3("track306.mp3");
        break;
      case 97:
        MP3player.playMP3("track307.mp3");
        break;
      case 98:
        MP3player.playMP3("track308.mp3");
        break;
      case 99:
        MP3player.playMP3("track309.mp3");
        break;
      case 100:
        MP3player.playMP3("track310.mp3");
        break;
    }
  }
}
