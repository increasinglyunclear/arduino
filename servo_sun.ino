// Converts realtime sun position data from Processing to servo position
// for my Sun installation, v.1
// Kevin Walker 26 Jun 2010

#include <Servo.h>
Servo servo1; 
int incomingByte; // for incoming serial data
int ledPin = 13; // pin for the LED 
int pos = 0;    // variable to store the servo position 

void setup() {

  pinMode(ledPin, OUTPUT);  // ledPin is as an OUTPUT 
  servo1.attach(15); //analog pin 1
  digitalWrite(ledPin, HIGH);  // turn the LED on 
  Serial.begin(115200);

}

void loop() {

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("received ");
    Serial.println(incomingByte);
    if (incomingByte == 'r') {
      int reading = servo1.read();
      Serial.print("servo ");
      Serial.println(reading);
      if (reading < 180) {
        for(pos = reading; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
          servo1.write(pos);              // tell servo to go to position in variable 'pos' 
          delay(30);                       // waits 30ms for the servo to reach the position 
        } 
      }
    }
    else
    {
      servo1.write(incomingByte);
    }

//    if (incomingByte == '0') {
//      digitalWrite(ledPin, LOW);  // turn the LED off 
//      servo1.write(90);
//      Serial.println("zero");
//    }
//
//    if (incomingByte == '1') {
//      digitalWrite(ledPin, HIGH);  // turn the LED on 
//      servo1.write(0);
//      Serial.println("one");
//    }

  }
}
