/*
For testing XBee via serial
Kevin Walker 16 Mar 2014
Based on Blink
 */

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);   
  Serial.begin(9600);  
}

// the loop routine runs over and over again forever:
void loop() {

  if (Serial.available() > 0) {
    if (Serial.read() == 'D'){ //ring the bell briefly 
    digitalWrite(led, HIGH); 
    delay(1000); 
    digitalWrite(led, LOW);
    } 
  }

//  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);               // wait for a second
//  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);               // wait for a second
}


