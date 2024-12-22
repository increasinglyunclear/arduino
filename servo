
// Ultrasonic Servo: Maxbotix EZ1 ultrasonic rangefinder
// controlling a 180 degree servo motor 
// Code adapted from Sweep example sketch
// by Kevin Walker Jan 2013 

#include <Servo.h> 
Servo myservo;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 
int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo.attach(15);  // attaches the servo on pin 9 to the servo object 
}

void loop() {
  pinMode(7, INPUT); // set Pin 7 as input from sensor
  int pulse = pulseIn(7, HIGH); // send a pulse out Pin 7
  int inches = pulse/147; // convert the pulse. 147 microseconds = 1 inch
  //Serial.print(inches); // send the inches reading into the serial port for monitoring
  //Serial.println(); // print an empty line
  int reading = map(inches, 0, 96, 0, 180);
  Serial.print(reading); // send the inches reading into the serial port for monitoring
  Serial.println(); // print an empty line
  if (inches < 10) {
    if (pos < 180) {
      for(pos = 0; pos < 180; pos += 2)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
  }
  else {
    if (pos > 1) {
      for(pos = 180; pos>=1; pos-=2)     // goes from 180 degrees to 0 degrees 
      {                                
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
  }
  delay(500);
}














