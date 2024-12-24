// Code for testing photocell, adapted from 'Analog Read to LED' by David Cuartielles
// Kevin Walker 03 Feb 2010

int sensorPin = 0; // input pin for the photocell
int ledPin = 13;   // pin for the LED 
int val = 0;       // variable to store the input 

void setup() { 
  pinMode(ledPin, OUTPUT);  // ledPin is as an OUTPUT 
  Serial.begin(115200);
} 
void loop() { 
  val = analogRead(sensorPin); // read the value from 
  // the sensor 
  val = val/3;

  digitalWrite(ledPin, HIGH);  // turn the LED on 
  delay(300);                  // stop the program for 
  // some time 
  digitalWrite(ledPin, LOW);   // turn the LED off 
  delay(300);                  // stop the program for 
  // some time 
  Serial.println(val);
}
