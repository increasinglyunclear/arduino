// ARDUINO CODE TO CONTROL ADAFRUIT LED PIXELS
// KEVIN WALKER 2013


#include "SPI.h"
#include "Adafruit_WS2801.h"

int dataPin  = 23;    // Yellow wire to pin 2
int clockPin = 24;    // Green wire to pin 3

// Initialise: put in the number of lights in your strip.
Adafruit_WS2801 strip = Adafruit_WS2801(25, dataPin, clockPin);

void setup() {
  strip.begin();

  //strip.setPixelColor(5, Color(255, 0, 0)); // FOR TESTING: Control one light - numbered starting 0

  // Control x number of lights:
    int x;
    for (x = 0; x < 25; x++) {
      strip.setPixelColor(x, Color(255, 255, 0));
    }

  strip.show();
}


void loop() {
  strip.begin();

  // TEST: CONTROL ONE PIXEL. 
  //strip.setPixelColor(0, Color(0, 0, 0)); // Control one light - numbered starting 0

  // Control x number of lights:
    int x;
    for (x = 0; x < 50; x++) { // PUT IN THE NUMBER OF PIXELS TO TURN ON
      strip.setPixelColor(x, Color(255, 255, 255));
    }

  strip.show();
}


/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}
