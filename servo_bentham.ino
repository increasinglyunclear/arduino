// random servo sweep
// For eyeball installation https://increasinglyunclear.world/panopticon/
// Kevin Walker 19 Aug 2013

#include <Servo.h> 
Servo myservo; 
int pos = 0;    // variable to store the servo position 
int incomingByte; // for incoming serial data

void setup() 
{ 
  myservo.attach(15);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); 
  myservo.write(0);      
} 


void loop() 
{ 

  //if (Serial.available() > 0) {
  // incomingByte = Serial.read();
  // Serial.println(incomingByte);

  int sensorValue = analogRead(A0);
  int sensorValue2 = analogRead(A2);

  if (sensorValue > sensorValue2) {
    if (pos < 180){
      for(pos = 1; pos < 180; pos += 4)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(30);                       // waits 15ms for the servo to reach the position 
      } 
    }
  }

  // delay(random(10000));

  if (sensorValue2 > sensorValue) {
    if (pos > 1){
      for(pos = 180; pos>=1; pos-=4)     // goes from 180 degrees to 0 degrees 
      {                                
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(30);                       // waits 15ms for the servo to reach the position 
      } 
    }
  }

  if (sensorValue2 == sensorValue) {
    //for(pos = 30; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
    // {                                
    myservo.write(90);              // tell servo to go to position in variable 'pos' 
    delay(30);                       // waits 15ms for the servo to reach the position 
    // } 
  }


  //}
} 










