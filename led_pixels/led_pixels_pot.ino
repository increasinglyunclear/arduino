// This is for a bedside lamp, using two strands of Adafruit LED Pixels
// colour control using potentiometer
// https://www.instagram.com/p/CGYBXAQh3dG/?utm_source=ig_web_copy_link&igsh=MzRlODBiNWFlZA==
// Kevin Walker 12 Oct 2020

#include "SPI.h"
#include "Adafruit_WS2801.h"

int dataPin  = 2;    // Yellow wire to pin 2
int clockPin = 3;    // Green wire to pin 3
int potPin = A0;
int val = 0;

// Initialise: put in the number of lights in your strip.
Adafruit_WS2801 strip = Adafruit_WS2801(50, dataPin, clockPin);

void setup() {
  strip.begin();
  strip.show();
  Serial.begin(9600);
}


void loop() {
  val = analogRead(potPin);
  int val2 = (abs(1024 - val));
  int val3 = map(val2, 0, 1024, 0, 765);

  int b = val3 + 512;
  if (val3 < 512) {
    b = 0;
  } else {
    b = (val3 - 512);
  }

  int g = val3 - 255;
  if (val3 > 512) {
    g = 255;
  }
  if (g < 0) {
    g = 0;
  }

  int r = val3;
  if (r > 255) {
    r = 255;
  }
  if (r < 10) {
    r = 0;
  }

  Serial.println(r);

  strip.begin();
  // Control x number of lights:
  int x;
  for (x = 0; x < 50; x++) {
    strip.setPixelColor(x, Color(r, g, b));
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
