// EZ1 Ultrasonic rangefinder as input
// Tiny piezo speaker as output
// KEVIN WALKER 2012

// EZ1 PWR to Arduino 5V
// EZ1 GND to Arduino Gnd
// EZ1 PW to Arduino Pin 7

// Speaker goes to Arduino pin 12 and Gnd

void setup() {
  pinMode(7, INPUT); // Pin 7 takes data from EZ1
  pinMode(10, OUTPUT); // Pin 12 is data out to speaker
}

void loop() {
  //int pulse = pulseIn(7, HIGH); // Send a pulse to EZ1
  tone(10, 0) ; // Send what you get back to speaker; adjust the pulse as needed to get the range you want.
}
