// Plays random file from MP3 shield
// created for 'F0lded 1n' installation http://increasinglyunclear.world/folded/
// Kevin Walker 22 Sep 2022

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

int16_t last_ms_char; // milliseconds of last recieved character from Serial port.
int8_t buffer_pos; // next position to recieve character from Serial port.
char buffer[6]; // 0-35K+null

uint8_t result; //result code from some function as to be tested at later time.

long randNumber;
String index;

void setup() {

  Serial.begin(115200);
  Serial.print(F("F_CPU = "));
  Serial.println(F_CPU);
  Serial.print(F("Free RAM = ")); // available in Version 1.0 F() bases the string to into Flash, to use less SRAM.
  Serial.print(FreeStack(), DEC);  // FreeRam() is provided by SdFatUtil.h
  Serial.println(F(" Should be a base line of 1017, on ATmega328 when using INTx"));

  //Initialize the SdCard.
  if (!sd.begin(SD_SEL, SPI_FULL_SPEED)) sd.initErrorHalt();
  if (!sd.chdir("/")) sd.errorHalt("sd.chdir");

  //Initialize the MP3 Player Shield
  result = MP3player.begin();
  if (result != 0) {
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to start MP3 player"));
    if ( result == 6 ) {
      Serial.println(F("Warning: patch file not found, skipping.")); // can be removed for space, if needed.
      Serial.println(F("Use the \"d\" command to verify SdCard can be read")); // can be removed for space, if needed.
    }
  }

  //  help();
  last_ms_char = millis(); // stroke the inter character timeout.
  buffer_pos = 0; // start the command string at zero length.
  //  parse_menu('l'); // display the list of files to play

  //result = MP3player.playMP3("0001.mp3");
  MP3player.playTrack(00001);
}

void loop() {

  if ( ! MP3player.isPlaying() )
    play_random_file();

  //  if (!MP3player.isPlaying()) {
  //    randNumber = random(3) + 1;
  //    if (randNumber = 1) {
  //      MP3player.playMP3("0001.mp3");
  //    }
  //    else if (randNumber = 2) {
  //      MP3player.playMP3("0002.mp3");
  //    }
  //    else if (randNumber = 3) {
  //      MP3player.playMP3("0003.mp3");
  //    }
  //  }

}

void play_random_file()
{
  char filename[20];
  int count = 0;

  // Count the files.
  SdFile file;
  sd.chdir("/", true);
  while (file.openNext(sd.vwd(), O_READ)) {
    count += 1;
    file.close();
  }

  Serial.println( filename );

  //    // Pick a  random one
  //    int r = random( 0, count + 1);
  //
  //    // Loop over all the files a second time - if our current
  //    // index equals the random number then that's the one we'll
  //    // play.
  //    sd.chdir("/", true);
  //    count = 0;
  //    while (file.openNext(sd.vwd(), O_READ)) {
  //      if ( count == r )
  //      {
  //        file.getFilename(filename);
  //        Serial.print( "Playing the random filename " );
  //        Serial.println( filename );
  //        int8_t result = MP3player.playMP3(filename);
  //        if (result != 0) {
  //          Serial.print(F("Error code: "));
  //          Serial.print(result);
  //          Serial.println(F(" when trying to play the track!"));
  //        }
  //
  //      }
  //      file.close();
  //      count += 1;
  //    }
}

uint32_t  millis_prv;

//------------------------------------------------------------------------------
/**
   \brief Decode the Menu.

   Parses through the characters of the users input, executing corresponding
   MP3player library functions and features then displaying a brief menu and
   prompting for next input command.
*/
void parse_menu(byte key_command) {

  uint8_t result; // result code from some function as to be tested at later time.

  // Note these buffer may be desired to exist globably.
  // but do take much space if only needed temporarily, hence they are here.
  char title[30]; // buffer to contain the extract the Title from the current filehandles
  char artist[30]; // buffer to contain the extract the artist name from the current filehandles
  char album[30]; // buffer to contain the extract the album name from the current filehandles

  Serial.print(F("Received command: "));
  Serial.write(key_command);
  Serial.println(F(" "));

  //if s, stop the current track
  if (key_command == 's') {
    Serial.println(F("Stopping"));
    MP3player.stopTrack();

    //if 1-9, play corresponding track
  } else if (key_command >= '1' && key_command <= '9') {
    //convert ascii numbers to real numbers
    key_command = key_command - 48;

#if USE_MULTIPLE_CARDS
    sd.chvol(); // assign desired sdcard's volume.
#endif
    //tell the MP3 Shield to play a track
    result = MP3player.playTrack(key_command);

    //check result, see readme for error codes.
    if (result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to play track"));
    } else {

      Serial.println(F("Playing:"));

      //we can get track info by using the following functions and arguments
      //the functions will extract the requested information, and put it in the array we pass in
      MP3player.trackTitle((char*)&title);
      MP3player.trackArtist((char*)&artist);
      MP3player.trackAlbum((char*)&album);

      //print out the arrays of track information
      Serial.write((byte*)&title, 30);
      Serial.println();
      Serial.print(F("by:  "));
      Serial.write((byte*)&artist, 30);
      Serial.println();
      Serial.print(F("Album:  "));
      Serial.write((byte*)&album, 30);
      Serial.println();
    }

    //if +/- to change volume
  } else if ((key_command == '-') || (key_command == '+')) {
    union twobyte mp3_vol; // create key_command existing variable that can be both word and double byte of left and right.
    mp3_vol.word = MP3player.getVolume(); // returns a double uint8_t of Left and Right packed into int16_t

    if (key_command == '-') { // note dB is negative
      // assume equal balance and use byte[1] for math
      if (mp3_vol.byte[1] >= 254) { // range check
        mp3_vol.byte[1] = 254;
      } else {
        mp3_vol.byte[1] += 2; // keep it simpler with whole dB's
      }
    } else {
      if (mp3_vol.byte[1] <= 2) { // range check
        mp3_vol.byte[1] = 2;
      } else {
        mp3_vol.byte[1] -= 2;
      }
    }
    // push byte[1] into both left and right assuming equal balance.
    MP3player.setVolume(mp3_vol.byte[1], mp3_vol.byte[1]); // commit new volume
    Serial.print(F("Volume changed to -"));
    Serial.print(mp3_vol.byte[1] >> 1, 1);
    Serial.println(F("[dB]"));

    //if < or > to change Play Speed
  } else if ((key_command == '>') || (key_command == '<')) {
    uint16_t playspeed = MP3player.getPlaySpeed(); // create key_command existing variable
    // note playspeed of Zero is equal to ONE, normal speed.
    if (key_command == '>') { // note dB is negative
      // assume equal balance and use byte[1] for math
      if (playspeed >= 254) { // range check
        playspeed = 5;
      } else {
        playspeed += 1; // keep it simpler with whole dB's
      }
    } else {
      if (playspeed == 0) { // range check
        playspeed = 0;
      } else {
        playspeed -= 1;
      }
    }
    MP3player.setPlaySpeed(playspeed); // commit new playspeed
    Serial.print(F("playspeed to "));
    Serial.println(playspeed, DEC);

    /* Alterativly, you could call a track by it's file name by using playMP3(filename);
      But you must stick to 8.1 filenames, only 8 characters long, and 3 for the extension */
  } else if (key_command == 'f' || key_command == 'F') {
    uint32_t offset = 0;
    if (key_command == 'F') {
      offset = 2000;
    }

    //create a string with the filename
    char trackName[] = "track001.mp3";

#if USE_MULTIPLE_CARDS
    sd.chvol(); // assign desired sdcard's volume.
#endif
    //tell the MP3 Shield to play that file
    result = MP3player.playMP3(trackName, offset);
    //check result, see readme for error codes.
    if (result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to play track"));
    }

    /* Display the file on the SdCard */
  } else if (key_command == 'd') {
    if (!MP3player.isPlaying()) {
      // prevent root.ls when playing, something locks the dump. but keeps playing.
      // yes, I have tried another unique instance with same results.
      // something about SdFat and its 500byte cache.
      Serial.println(F("Files found (name date time size):"));
      sd.ls(LS_R | LS_DATE | LS_SIZE);
    } else {
      Serial.println(F("Busy Playing Files, try again later."));
    }

    /* Get and Display the Audio Information */
  } else if (key_command == 'i') {
    MP3player.getAudioInfo();

  } else if (key_command == 'p') {
    if ( MP3player.getState() == playback) {
      MP3player.pauseMusic();
      Serial.println(F("Pausing"));
    } else if ( MP3player.getState() == paused_playback) {
      MP3player.resumeMusic();
      Serial.println(F("Resuming"));
    } else {
      Serial.println(F("Not Playing!"));
    }

  } else if (key_command == 't') {
    int8_t teststate = MP3player.enableTestSineWave(126);
    if (teststate == -1) {
      Serial.println(F("Un-Available while playing music or chip in reset."));
    } else if (teststate == 1) {
      Serial.println(F("Enabling Test Sine Wave"));
    } else if (teststate == 2) {
      MP3player.disableTestSineWave();
      Serial.println(F("Disabling Test Sine Wave"));
    }

  } else if (key_command == 'S') {
    Serial.println(F("Current State of VS10xx is."));
    Serial.print(F("isPlaying() = "));
    Serial.println(MP3player.isPlaying());

    Serial.print(F("getState() = "));
    switch (MP3player.getState()) {
      case uninitialized:
        Serial.print(F("uninitialized"));
        break;
      case initialized:
        Serial.print(F("initialized"));
        break;
      case deactivated:
        Serial.print(F("deactivated"));
        break;
      case loading:
        Serial.print(F("loading"));
        break;
      case ready:
        Serial.print(F("ready"));
        break;
      case playback:
        Serial.print(F("playback"));
        break;
      case paused_playback:
        Serial.print(F("paused_playback"));
        break;
      case testing_memory:
        Serial.print(F("testing_memory"));
        break;
      case testing_sinewave:
        Serial.print(F("testing_sinewave"));
        break;
    }
    Serial.println();

  } else if (key_command == 'b') {
    Serial.println(F("Playing Static MIDI file."));
    MP3player.SendSingleMIDInote();
    Serial.println(F("Ended Static MIDI file."));

#if !defined(__AVR_ATmega32U4__)
  } else if (key_command == 'm') {
    uint16_t teststate = MP3player.memoryTest();
    if (teststate == -1) {
      Serial.println(F("Un-Available while playing music or chip in reset."));
    } else if (teststate == 2) {
      teststate = MP3player.disableTestSineWave();
      Serial.println(F("Un-Available while Sine Wave Test"));
    } else {
      Serial.print(F("Memory Test Results = "));
      Serial.println(teststate, HEX);
      Serial.println(F("Result should be 0x83FF."));
      Serial.println(F("Reset is needed to recover to normal operation"));
    }

  } else if (key_command == 'e') {
    uint8_t earspeaker = MP3player.getEarSpeaker();
    if (earspeaker >= 3) {
      earspeaker = 0;
    } else {
      earspeaker++;
    }
    MP3player.setEarSpeaker(earspeaker); // commit new earspeaker
    Serial.print(F("earspeaker to "));
    Serial.println(earspeaker, DEC);

  } else if (key_command == 'r') {
    MP3player.resumeMusic(2000);

  } else if (key_command == 'R') {
    MP3player.stopTrack();
    MP3player.vs_init();
    Serial.println(F("Reseting VS10xx chip"));

  } else if (key_command == 'g') {
    int32_t offset_ms = 20000; // Note this is just an example, try your own number.
    Serial.print(F("jumping to "));
    Serial.print(offset_ms, DEC);
    Serial.println(F("[milliseconds]"));
    result = MP3player.skipTo(offset_ms);
    if (result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to skip track"));
    }

  } else if (key_command == 'k') {
    int32_t offset_ms = -1000; // Note this is just an example, try your own number.
    Serial.print(F("moving = "));
    Serial.print(offset_ms, DEC);
    Serial.println(F("[milliseconds]"));
    result = MP3player.skip(offset_ms);
    if (result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to skip track"));
    }

  } else if (key_command == 'O') {
    MP3player.end();
    Serial.println(F("VS10xx placed into low power reset mode."));

  } else if (key_command == 'o') {
    MP3player.begin();
    Serial.println(F("VS10xx restored from low power reset mode."));

  } else if (key_command == 'D') {
    uint16_t diff_state = MP3player.getDifferentialOutput();
    Serial.print(F("Differential Mode "));
    if (diff_state == 0) {
      MP3player.setDifferentialOutput(1);
      Serial.println(F("Enabled."));
    } else {
      MP3player.setDifferentialOutput(0);
      Serial.println(F("Disabled."));
    }

  } else if (key_command == 'V') {
    MP3player.setVUmeter(1);
    Serial.println(F("Use \"No line ending\""));
    Serial.print(F("VU meter = "));
    Serial.println(MP3player.getVUmeter());
    Serial.println(F("Hit Any key to stop."));

    while (!Serial.available()) {
      union twobyte vu;
      vu.word = MP3player.getVUlevel();
      Serial.print(F("VU: L = "));
      Serial.print(vu.byte[1]);
      Serial.print(F(" / R = "));
      Serial.print(vu.byte[0]);
      Serial.println(" dB");
      delay(1000);
    }
    Serial.read();

    MP3player.setVUmeter(0);
    Serial.print(F("VU meter = "));
    Serial.println(MP3player.getVUmeter());

  } else if (key_command == 'T') {
    uint16_t TrebleFrequency = MP3player.getTrebleFrequency();
    Serial.print(F("Former TrebleFrequency = "));
    Serial.println(TrebleFrequency, DEC);
    if (TrebleFrequency >= 15000) { // Range is from 0 - 1500Hz
      TrebleFrequency = 0;
    } else {
      TrebleFrequency += 1000;
    }
    MP3player.setTrebleFrequency(TrebleFrequency);
    Serial.print(F("New TrebleFrequency = "));
    Serial.println(MP3player.getTrebleFrequency(), DEC);

  } else if (key_command == 'E') {
    int8_t TrebleAmplitude = MP3player.getTrebleAmplitude();
    Serial.print(F("Former TrebleAmplitude = "));
    Serial.println(TrebleAmplitude, DEC);
    if (TrebleAmplitude >= 7) { // Range is from -8 - 7dB
      TrebleAmplitude = -8;
    } else {
      TrebleAmplitude++;
    }
    MP3player.setTrebleAmplitude(TrebleAmplitude);
    Serial.print(F("New TrebleAmplitude = "));
    Serial.println(MP3player.getTrebleAmplitude(), DEC);

  } else if (key_command == 'B') {
    uint16_t BassFrequency = MP3player.getBassFrequency();
    Serial.print(F("Former BassFrequency = "));
    Serial.println(BassFrequency, DEC);
    if (BassFrequency >= 150) { // Range is from 20hz - 150hz
      BassFrequency = 0;
    } else {
      BassFrequency += 10;
    }
    MP3player.setBassFrequency(BassFrequency);
    Serial.print(F("New BassFrequency = "));
    Serial.println(MP3player.getBassFrequency(), DEC);

  } else if (key_command == 'C') {
    uint16_t BassAmplitude = MP3player.getBassAmplitude();
    Serial.print(F("Former BassAmplitude = "));
    Serial.println(BassAmplitude, DEC);
    if (BassAmplitude >= 15) { // Range is from 0 - 15dB
      BassAmplitude = 0;
    } else {
      BassAmplitude++;
    }
    MP3player.setBassAmplitude(BassAmplitude);
    Serial.print(F("New BassAmplitude = "));
    Serial.println(MP3player.getBassAmplitude(), DEC);

  } else if (key_command == 'M') {
    uint16_t monostate = MP3player.getMonoMode();
    Serial.print(F("Mono Mode "));
    if (monostate == 0) {
      MP3player.setMonoMode(1);
      Serial.println(F("Enabled."));
    } else {
      MP3player.setMonoMode(0);
      Serial.println(F("Disabled."));
    }
#endif

    /* List out music files on the SdCard */
  } else if (key_command == 'l') {
    if (!MP3player.isPlaying()) {
      Serial.println(F("Music Files found :"));
      SdFile file;
      char filename[13];
      sd.chdir("/", true);
      uint16_t count = 1;
      while (file.openNext(sd.vwd(), O_READ))
      {
        file.getName(filename, sizeof(filename));
        if ( isFnMusic(filename) ) {
          SerialPrintPaddedNumber(count, 5 );
          Serial.print(F(": "));
          Serial.println(filename);
          count++;
        }
        file.close();
      }
      Serial.println(F("Enter Index of File to play"));

    } else {
      Serial.println(F("Busy Playing Files, try again later."));
    }

  } else if (key_command == 'h') {
    help();
  }

  // print prompt after key stroke has been processed.
  Serial.print(F("Time since last command: "));
  Serial.println((float) (millis() -  millis_prv) / 1000, 2);
  millis_prv = millis();
  Serial.print(F("Enter s,1-9,+,-,>,<,f,F,d,i,p,t,S,b"));
#if !defined(__AVR_ATmega32U4__)
  Serial.print(F(",m,e,r,R,g,k,O,o,D,V,B,C,T,E,M:"));
#endif
  Serial.println(F(",l,h :"));
}

//------------------------------------------------------------------------------
/**
   \brief Print Help Menu.

   Prints a full menu of the commands available along with descriptions.
*/
void help() {
  Serial.println(F("Arduino SFEMP3Shield Library Example:"));
  Serial.println(F(" courtesy of Bill Porter & Michael P. Flaga"));
  Serial.println(F("COMMANDS:"));
  Serial.println(F(" [1-9] to play a track"));
  Serial.println(F(" [f] play track001.mp3 by filename example"));
  Serial.println(F(" [F] same as [f] but with initial skip of 2 second"));
  Serial.println(F(" [s] to stop playing"));
  Serial.println(F(" [d] display directory of SdCard"));
  Serial.println(F(" [+ or -] to change volume"));
  Serial.println(F(" [> or <] to increment or decrement play speed by 1 factor"));
  Serial.println(F(" [i] retrieve current audio information (partial list)"));
  Serial.println(F(" [p] to pause."));
  Serial.println(F(" [t] to toggle sine wave test"));
  Serial.println(F(" [S] Show State of Device."));
  Serial.println(F(" [b] Play a MIDI File Beep"));
#if !defined(__AVR_ATmega32U4__)
  Serial.println(F(" [e] increment Spatial EarSpeaker, default is 0, wraps after 4"));
  Serial.println(F(" [m] perform memory test. reset is needed after to recover."));
  Serial.println(F(" [M] Toggle between Mono and Stereo Output."));
  Serial.println(F(" [g] Skip to a predetermined offset of ms in current track."));
  Serial.println(F(" [k] Skip a predetermined number of ms in current track."));
  Serial.println(F(" [r] resumes play from 2s from begin of file"));
  Serial.println(F(" [R] Resets and initializes VS10xx chip."));
  Serial.println(F(" [O] turns OFF the VS10xx into low power reset."));
  Serial.println(F(" [o] turns ON the VS10xx out of low power reset."));
  Serial.println(F(" [D] to toggle SM_DIFF between inphase and differential output"));
  Serial.println(F(" [V] Enable VU meter Test."));
  Serial.println(F(" [B] Increament bass frequency by 10Hz"));
  Serial.println(F(" [C] Increament bass amplitude by 1dB"));
  Serial.println(F(" [T] Increament treble frequency by 1000Hz"));
  Serial.println(F(" [E] Increament treble amplitude by 1dB"));
#endif
  Serial.println(F(" [l] Display list of music files"));
  Serial.println(F(" [0####] Enter index of file to play, zero pad! e.g. 01-65534"));
  Serial.println(F(" [h] this help"));
}

void SerialPrintPaddedNumber(int16_t value, int8_t digits ) {
  int currentMax = 10;
  for (byte i = 1; i < digits; i++) {
    if (value < currentMax) {
      Serial.print("0");
    }
    currentMax *= 10;
  }
  Serial.print(value);
}
