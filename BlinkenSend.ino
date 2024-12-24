/*
 * Based on Blink: The basic Arduino example.  
 * Turns on an LED on and off for one second each,
 * and sends the LED state into the computer via serial USB.
 * Open the Serial Monitor in Arduino software to see incoming data.
 * Kevin Walker Feb 2013
 */

void setup()                  
{
  pinMode(13, OUTPUT);      // sets pin 13 as output
  Serial.begin(115200);    // begin serial 
}

void loop()                     
{
  digitalWrite(13, HIGH);   // turns the LED on
  Serial.println("on");
  delay(1000);                  // waits for a second
  
  digitalWrite(13, LOW);    // turns the LED off
  Serial.println("off");
  delay(1000);                  // waits for a second
}
