// My bedside light - an Adafruit DotStarMatrix 8x32 display
// I had to dial down the brightness in the code to drive the whole display with an Uno
// Kevin Walker 11 Dec 2024

#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_GFX.h>
#include <Adafruit_DotStar.h>
#include <gamma.h>
#include <SPI.h>

#define DATAPIN  4 // (blue wire)
#define CLOCKPIN 5 // (green)

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
                                  32, 8, DATAPIN, CLOCKPIN,
                                  DS_MATRIX_BOTTOM     + DS_MATRIX_RIGHT +
                                  DS_MATRIX_COLUMNS + DS_MATRIX_ZIGZAG,
                                  DOTSTAR_BRG);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255)
};

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(20); // had to dial this down due to power requirements
  matrix.setTextColor(colors[255]);
}

int x    = matrix.width();
int pass = 0;

void loop() {
  // Not sure why but red and green are reversed. This is 'warm white' according to ChatGPT:
  matrix.fillScreen(matrix.Color(200, 255, 100)); // comment out for static.
  // matrix.setCursor(x, 0); // xet to x,0 for scrolling, 0,0 static
  // matrix.print(F("Howdy"));

  // scrolling:
  // if (--x < -36) {
  //   x = matrix.width();
  //   if (++pass >= 3) pass = 0;
  //   matrix.setTextColor(colors[pass]);
  // }

  matrix.show();
  //delay(100);
}
