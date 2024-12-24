// Generates random number 0-299, uses to build filename
// for 'F0lded 1n' installation https://increasinglyunclear.world/folded/
// Kevin Walker 25 Sep 2022

long randNumber;
String index;

void setup() {
  Serial.begin(115200);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
}

void loop() {
  // print a random number from 0 to 299
  randNumber = random(3) + 1;
  if (randNumber < 10) {
    index = "000" + String(randNumber) + ".mp3";
    Serial.println(index);
    //  } else if (randNumber < 100) {
    //    index = "00" + String(randNumber);
    //  } else {
    //    index = String(randNumber);
  }

  delay(50);
}
