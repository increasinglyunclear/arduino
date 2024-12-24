// Sends ADXL377 accelerometer data over wifi
// commissioned by National Football Museum for RCA Space Program 'Football and Physics' project
// Kevin Walker 20 Feb 2015. Adapted from:

/******************************************************************************
 * ADXL377_example.ino
 * Simple example for using the ADXL377 accelerometer breakout.
 * Jordan McConnell @ SparkFun Electronics
 * 3 July 2014
 * https://github.com/sparkfun/ADXL377_Breakout
 * This example collects raw accelerometer data from the ADXL377 sensor using
 * analog reads and then converts it into common units (g's) and prints the
 * result to the Serial Monitor.
 * Developed/Tested with:
 * Arduino Uno
 * Arduino IDE 1.0.5
 * This code is beerware.
 * Distributed as-is; no warranty is given. 
 ******************************************************************************/

// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data

// WIFLY STUFF:
#include <WiFlyHQ.h>
#include <SoftwareSerial.h>
SoftwareSerial wifiSerial(8,9); // This is where Arduino i/o pins are set
WiFly wifly;
int timer = millis();

/* Change these to match your WiFi network */
const char mySSID[] = "kPhone";
const char myPassword[] = "S@cco471";
void terminal();
// END WIFLY STUFF

void setup()
{
  // Initialize serial communication at 115200 baud
  char buf[32];
  Serial.begin(115200);

  //WIFLY STUFF:
  wifiSerial.begin(9600);
  if (!wifly.begin(&wifiSerial, &Serial)) {
    Serial.println("Failed to start wifly");
    terminal();
  }

  /* Join wifi network if not already associated */
  if (!wifly.isAssociated()) {
    /* Setup the WiFly to connect to a wifi network */
    Serial.println("Joining network");
    wifly.setSSID(mySSID);
    wifly.setPassphrase(myPassword);
    wifly.enableDHCP();
    if (wifly.join()) {
      Serial.println("Joined wifi network");
    } 
    else {
      Serial.println("Failed to join wifi network");
      terminal();
    }
  } 
  else {
    Serial.println("Already joined network");
  }

  //Serial.print("MAC: ");
  //Serial.println(wifly.getMAC(buf, sizeof(buf)));
  Serial.print("IP: ");
  Serial.println(wifly.getIP(buf, sizeof(buf)));
  //Serial.print("Netmask: ");
  //Serial.println(wifly.getNetmask(buf, sizeof(buf)));
  //Serial.print("Gateway: ");
  //Serial.println(wifly.getGateway(buf, sizeof(buf)));
  //wifly.setDeviceID("Wifly-WebClient");
  //Serial.print("DeviceID: ");
  //Serial.println(wifly.getDeviceID(buf, sizeof(buf)));

  if (wifly.isConnected()) {
    Serial.println("Old connection active. Closing");
    wifly.close();
  }

  //  if (wifly.open(site, 80)) {
  //    Serial.print("Connected to ");
  //    Serial.println(site);
  //
  //    /* Send the request */
  //    wifly.println("GET /happenstance/time.php HTTP/1.1\nHost:walkerred.com\n\n");
  //  } 
  //  else {
  //    Serial.println("Failed to connect");
  //  }
  //END WIFLY STUF

}

// Read, scale, and print accelerometer data
void loop()
{
  // Get raw accelerometer data for each axis
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);
  int rawZ = analogRead(A2);

  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  if (micro_is_5V) // Microcontroller runs off 5V
  {
    scaledX = mapf(rawX, 0, 675, -scale, scale); // 3.3/5 * 1023 =~ 675
    scaledY = mapf(rawY, 0, 675, -scale, scale);
    scaledZ = mapf(rawZ, 0, 675, -scale, scale);
  }
  else // Microcontroller runs off 3.3V
  {
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
  }

  // Print out raw X,Y,Z accelerometer readings
  //  wifly.print("X: "); 
  //  wifly.println(rawX);
  //  wifly.print("Y: "); 
  //  wifly.println(rawY);
  //  wifly.print("Z: "); 
  //  wifly.println(rawZ);
  //  wifly.println();

  // Print out scaled X,Y,Z accelerometer readings
  //wifly.print("X: "); 
  wifly.println(scaledX); 
  //wifly.println(" g");
  //wifly.print("Y: "); 
  //wifly.print(scaledY); 
  //wifly.println(" g");
  //wifly.print("Z: "); 
  //wifly.print(scaledZ); 
  //wifly.println(" g");
  //wifly.println();

  delay(1000); // Minimum delay of 2 milliseconds between sensor reads (500 Hz)

  //  if (wifly.available() > 0) {
  //    char ch = wifly.read();
  //    Serial.write(ch);
  //    if (ch == '\n') {
  //      /* add a carriage return */
  //      Serial.write('\r');
  //    }
  //  }
  //
  //  if (Serial.available() > 0) {
  //    wifly.write(Serial.read());
  //  }
}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Connect the WiFly serial to the serial monitor. */
void terminal()
{
  while (1) {
    if (wifly.available() > 0) {
      Serial.write(wifly.read());
    }

  }
}




