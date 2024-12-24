/* RN-XV WiFly http client
* for pulling realtime TfL train data into my Old Street installation http://increasinglyunclear.world/old/
* Kevin Walker 26 Aug 2013
* 
* based on
 * WiFlyHQ Example httpclient.ino
 *
 * This sketch implements a simple Web client that connects to a 
 * web server, sends a GET, and then sends the result to the 
 * Serial monitor.
 *
 * Connect Wifly to Arduino as follows:
 * Wifly Pin 1 to Arduino 3.3V pin
 * Wifly Pin 2 to Arduino pin 8
 * Wifly Pin 3 to Arduino pin 9
 * Wifly Pin 10 to Arduino GND pin
 */

#include <WiFlyHQ.h>
#include <SoftwareSerial.h>
SoftwareSerial wifiSerial(8,9); // This is where Arduino i/o pins are set

WiFly wifly;

/* Change these to match your WiFi network */
const char mySSID[] = "BTHomeHub-8662";
const char myPassword[] = "39652247f6";
const char site[] = "www.tfl.gov.uk";
void terminal();
String webdata = "";

void setup()
{
  char buf[32];

  Serial.begin(115200);
  //Serial.println("Starting");
  //Serial.print("Free memory: ");
  //Serial.println(wifly.getFreeMemory(),DEC);

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

  //  Serial.print("MAC: ");
  //  Serial.println(wifly.getMAC(buf, sizeof(buf)));
  //  Serial.print("IP: ");
  //  Serial.println(wifly.getIP(buf, sizeof(buf)));
  //  Serial.print("Netmask: ");
  //  Serial.println(wifly.getNetmask(buf, sizeof(buf)));
  //  Serial.print("Gateway: ");
  //  Serial.println(wifly.getGateway(buf, sizeof(buf)));
  //  wifly.setDeviceID("Wifly-WebClient");
  //  Serial.print("DeviceID: ");
  //  Serial.println(wifly.getDeviceID(buf, sizeof(buf)));

  if (wifly.isConnected()) {
    Serial.println("Old connection active. Closing");
    wifly.close();
  }

  if (wifly.open(site, 80)) {
    Serial.print("Connected to ");
    Serial.println(site);

    /* Send the request */
    wifly.println("GET /tfl/livetravelnews/departure-boards/departureboards.aspx?mode=tube&station=OLD&line=northern HTTP/1.1\nHost:www.tfl.gov.uk\n\n");
  } 
  else {
    Serial.println("Failed to connect");
  }
}

void loop()
{
  if (wifly.available() > 0) {
    char ch = wifly.read();
    //if  (ch == 'B') {
    Serial.write(ch);
    //webdata = webdata + ch;
    //}
    if (ch == '\n') {
      /* add a carriage return */
      Serial.write('\r');
    }
  }

  int l = webdata.length();
  //if (l > 100) {
  if (webdata.substring(l) == "via Bank") {
    Serial.println(webdata);
  }

  if (Serial.available() > 0) {
    wifly.write(Serial.read());
  }
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











