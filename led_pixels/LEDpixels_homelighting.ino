// xmas lights 2024
// 2 (25 bulb) strands of Adafruit LED Pixels, all red
// driven by a Mega

#include "SPI.h"
#include "Adafruit_WS2801.h"
int dataPin  = 2;    // Yellow wire on Adafruit Pixels
int clockPin = 3;    // Green wire on Adafruit Pixels

// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
Adafruit_WS2801 strip = Adafruit_WS2801(50, dataPin, clockPin);

void setup() {
  strip.begin();
  strip.show();
  pinMode(11, OUTPUT);
}


void loop() {
  colorWipe(Color(255, 0, 0), 50);
}


void colorWipe(uint32_t c, uint8_t wait) {
  int i;
  int j;
  int r = random(255);
  int g = random(255);
  int b = random(255);
  int num = 5*random(17);

  tone(11, r*b);

  // for (i = 0; i < strip.numPixels(); i++) {

  for (j = num; j < num + 5; j++) {
    strip.setPixelColor(j, Color(r, g, b));
  }
  strip.show();
  delay(random(1000));

  for (j = num; j < num + 5; j++) {
    strip.setPixelColor(j, Color(0, 0, 0));
  }

  //CYCLE
  //    for (j = i; j < i + 5; j++) {
  //      strip.setPixelColor(j, Color(r, g, b));
  //    }
  //    strip.show();
  //    //delay(100);
  //
  //    for (j = i; j < i + 5; j++) {
  //      strip.setPixelColor(j, Color(0, 0, 0));
  //    }

  //strip.show();
  // delay(50);
  // }
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

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else {
    WheelPos -= 170;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
