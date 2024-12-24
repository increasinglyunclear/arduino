/* Simple motion sensor  
 * Sends data from photocell attached to hacked motion sensor light
 * for happenstanceproject.com
 * Kevin Walker 01 May 2012
 */

int sensorPin = 0; // input pin for the photocell 
int ledPin = 13;   // output pin for the LED 
int val = 0;       // variable for data from photocell

void setup() { 
  pinMode(ledPin, OUTPUT);  // ledPin is as an OUTPUT 
  Serial.begin(115200);
} 
void loop() { 
  val = analogRead(sensorPin); // read from photocell
  digitalWrite(ledPin, HIGH);  // turn the LED on 
  delay(500);   // stop the program for some time 
  digitalWrite(ledPin, LOW);   // turn the LED off 
  delay(500);
  if (val > 100)
  {
    Serial.println(val); 
    // EVERY LINE SENT = MOVEMENT FOR 1 sec.
    // VALUE INDICATES BATTERY LEVEL OF MOTION LEDs: 1024=full
  }


}




