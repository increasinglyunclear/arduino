// This is the installed 'story mode' of my Old Street installation https://increasinglyunclear.world/old/
// minus the live data - it tells a personal story.
// Note that LEDs are in groups of 5
// Kevin Walker 11 Jun 2013

#include "SPI.h"
#include "Adafruit_WS2801.h"

int dataPin  = 1;    // Yellow wire on Adafruit Pixels
int clockPin = 3;    // Green wire on Adafruit Pixels
Adafruit_WS2801 strip = Adafruit_WS2801(200, dataPin, clockPin); // NOTE NUMBER OF LEDs!

void setup() {
  strip.begin();
  strip.show();
}


void loop() {
  colorWipe(Color(255, 0, 0), 0);
}


void colorWipe(uint32_t c, uint8_t wait) {
  int i;
  int j;
  int k;
  int l;

  // sweep 1
  for (i=0; i < 199; i++) {
    strip.setPixelColor(i, Color(255, 255, 255));
    strip.setPixelColor(i+1, Color(255, 255, 255));
    strip.setPixelColor(i+2, Color(255, 255, 255));
    strip.setPixelColor(i+3, Color(255, 255, 255));
    strip.setPixelColor(i+4, Color(255, 255, 255));   
    strip.show();
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    //strip.show();
  }

  // TURN OFF LAST ONE
  for (i=195; i < 199; i++) {
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    strip.show();
  }

  delay(1000);

  // RANDOM SINGLE TILE
  j = random(40)*5*5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(1000);

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(random(2000));

  // SINGLE TILE
  j = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));

  delay(2000);

  // SWEEP PAIR 
  //j = random(40)*5;
  for (i=0; i < 199; i++) {
    strip.setPixelColor(i, Color(255, 255, 255));
    strip.setPixelColor(i+1, Color(255, 255, 255));
    strip.setPixelColor(i+2, Color(255, 255, 255));
    strip.setPixelColor(i+3, Color(255, 255, 255));
    strip.setPixelColor(i+4, Color(255, 255, 255));  
    strip.setPixelColor(i+5, Color(255, 255, 255));
    strip.setPixelColor(i+6, Color(255, 255, 255));
    strip.setPixelColor(i+7, Color(255, 255, 255));
    strip.setPixelColor(i+8, Color(255, 255, 255));
    strip.setPixelColor(i+9, Color(255, 255, 255));    
    strip.show();
    delay(0);
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    strip.setPixelColor(i+5, Color(0, 0, 0));
    strip.setPixelColor(i+6, Color(0, 0, 0));
    strip.setPixelColor(i+7, Color(0, 0, 0));
    strip.setPixelColor(i+8, Color(0, 0, 0));
    strip.setPixelColor(i+9, Color(0, 0, 0));
  }

  // TURN OFF LAST TWO
  for (i=195; i < 199; i++) {
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    strip.setPixelColor(i+5, Color(0, 0, 0));
    strip.setPixelColor(i+6, Color(0, 0, 0));
    strip.setPixelColor(i+7, Color(0, 0, 0));
    strip.setPixelColor(i+8, Color(0, 0, 0));
    strip.setPixelColor(i+9, Color(0, 0, 0));
    strip.show();
  }

  delay(2000);

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);
  strip.setPixelColor(j, Color(0, 0, 0));
  strip.setPixelColor(j+1, Color(0, 0, 0));
  strip.setPixelColor(j+2, Color(0, 0, 0));
  strip.setPixelColor(j+3, Color(0, 0, 0));
  strip.setPixelColor(j+4, Color(0, 0, 0));
  strip.setPixelColor(k, Color(0, 0, 0));
  strip.setPixelColor(k+1, Color(0, 0, 0));
  strip.setPixelColor(k+2, Color(0, 0, 0));
  strip.setPixelColor(k+3, Color(0, 0, 0));
  strip.setPixelColor(k+4, Color(0, 0, 0));

  delay(random(2000));

  // RANDOM DOUBLE TILE
  j = random(40)*5+5;
  k = random(40)*5+5;
  strip.setPixelColor(j, Color(255, 255, 255));
  strip.setPixelColor(j+1, Color(255, 255, 255));
  strip.setPixelColor(j+2, Color(255, 255, 255));
  strip.setPixelColor(j+3, Color(255, 255, 255));
  strip.setPixelColor(j+4, Color(255, 255, 255));   
  strip.setPixelColor(k, Color(255, 255, 255));
  strip.setPixelColor(k+1, Color(255, 255, 255));
  strip.setPixelColor(k+2, Color(255, 255, 255));
  strip.setPixelColor(k+3, Color(255, 255, 255));
  strip.setPixelColor(k+4, Color(255, 255, 255));   
  strip.show();
  delay(0);

  delay(2000);

  // SWEEP AWAY
  for (i=j; i < 199; i++) {
    strip.setPixelColor(i, Color(255, 255, 255));
    strip.setPixelColor(i+1, Color(255, 255, 255));
    strip.setPixelColor(i+2, Color(255, 255, 255));
    strip.setPixelColor(i+3, Color(255, 255, 255));
    strip.setPixelColor(i+4, Color(255, 255, 255));   
    strip.show();
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    //strip.show();
  }

  // TURN OFF LAST ONE
  for (i=195; i < 199; i++) {
    strip.setPixelColor(i, Color(0, 0, 0));
    strip.setPixelColor(i+1, Color(0, 0, 0));
    strip.setPixelColor(i+2, Color(0, 0, 0));
    strip.setPixelColor(i+3, Color(0, 0, 0));
    strip.setPixelColor(i+4, Color(0, 0, 0));
    strip.show();
  }

  if (k < j) {
    for (i=k; i < 199; i++) {
      strip.setPixelColor(i, Color(255, 255, 255));
      strip.setPixelColor(i+1, Color(255, 255, 255));
      strip.setPixelColor(i+2, Color(255, 255, 255));
      strip.setPixelColor(i+3, Color(255, 255, 255));
      strip.setPixelColor(i+4, Color(255, 255, 255));   
      strip.show();
      strip.setPixelColor(i, Color(0, 0, 0));
      strip.setPixelColor(i+1, Color(0, 0, 0));
      strip.setPixelColor(i+2, Color(0, 0, 0));
      strip.setPixelColor(i+3, Color(0, 0, 0));
      strip.setPixelColor(i+4, Color(0, 0, 0));
      //strip.show();
    }

    // TURN OFF LAST ONE
    for (i=195; i < 199; i++) {
      strip.setPixelColor(i, Color(0, 0, 0));
      strip.setPixelColor(i+1, Color(0, 0, 0));
      strip.setPixelColor(i+2, Color(0, 0, 0));
      strip.setPixelColor(i+3, Color(0, 0, 0));
      strip.setPixelColor(i+4, Color(0, 0, 0));
      strip.show();
    }
  }
  delay(2000);

}

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

















