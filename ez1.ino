// ARDUINO CODE TO CONTROL EZ1 ULTRASONIC SENSOR
// KEVIN WALKER 2012

int pwPin = 7;

void setup() {
  Serial.begin(115200);
}

void loop() {
  pinMode(7, INPUT);
  int pulse = pulseIn(7, HIGH);
  int inches = pulse / 147; //147uS per inch
  if (inches < 50) {
    Serial.print(inches);
    Serial.println();
    //delay(500);
  }
}






