/* RN-XV WiFly ad hoc server
* Kevin Walker 21 Apr 2013
 * based onWiFlyHQ Example httpclient.ino
 *
 * This sketch implements a simple server that sends data from
 * Arduino via serial.
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
//int timer = millis();

/* Change these to match your WiFi network */
//const char mySSID[] = "BTHomeHub-8662";
//const char myPassword[] = "39652247f6";
//void terminal();
void sendIndex();
void sendGreeting(char *name);
void send404();
char buf[80];


void setup()
{
 // char buf[32];
  // pinMode(7, INPUT); // Distance sensor
  //Serial.begin(115200);
  //Serial.println("Starting");
  //Serial.print("Free memory: ");
  //Serial.println(wifly.getFreeMemory(),DEC);

//  wifiSerial.begin(9600);
//  if (!wifly.begin(&wifiSerial, &Serial)) {
//    Serial.println("Failed to start wifly");
//    terminal();
//  }

  /* Join wifi network if not already associated */
  //if (!wifly.isAssociated()) {
  /* Setup the WiFly to connect to a wifi network */
  //Serial.println("Joining network");
  // wifly.setSSID(mySSID);
  // wifly.setPassphrase(myPassword);
  // wifly.enableDHCP();
  // if (wifly.join()) {
  //Serial.println("Joined wifi network");
  // } 
  // else {
  //Serial.println("Failed to join wifi network");
  //terminal();
  // }
  //} 
 // else {
  //  Serial.println("Already joined network");
  //}

  //wifly.setBroadcastInterval(0); // Turn off UPD broadcast
  //wifly.setDeviceID("Wifly-WebServer");
  wifly.setProtocol(WIFLY_PROTOCOL_TCP);
  if (wifly.getPort() != 80) {
    wifly.setPort(80);
    /* local port does not take effect until the WiFly has rebooted (2.32) */
    wifly.save();
    Serial.println(F("Set port to 80, rebooting to make it work"));
    wifly.reboot();
    delay(3000);
  }
  //Serial.println(F("Ready"));


  //Serial.print("MAC: ");
  //Serial.println(wifly.getMAC(buf, sizeof(buf)));
  //Serial.print("IP: ");
  //Serial.println(wifly.getIP(buf, sizeof(buf)));
  //Serial.print("Netmask: ");
  //Serial.println(wifly.getNetmask(buf, sizeof(buf)));
  //Serial.print("Gateway: ");
  //Serial.println(wifly.getGateway(buf, sizeof(buf)));
  //wifly.setDeviceID("Wifly-WebClient");
  //Serial.print("DeviceID: ");
  //Serial.println(wifly.getDeviceID(buf, sizeof(buf)));

//  if (wifly.isConnected()) {
//    Serial.println("Old connection active. Closing");
//    wifly.close();
//  }

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
}

void loop()
{

  if (wifly.available() > 0) {
    /* See if there is a request */
    if (wifly.gets(buf, sizeof(buf))) {
      if (strncmp_P(buf, PSTR("GET / "), 6) == 0) {
        /* GET request */
        //Serial.println(F("Got GET request"));
        while (wifly.gets(buf, sizeof(buf)) > 0) {
          /* Skip rest of request */
        }
        sendIndex();
        //Serial.println(F("Sent index page"));
      } 
      else if (strncmp_P(buf, PSTR("POST"), 4) == 0) {
        /* Form POST */
        char username[16];
        //Serial.println(F("Got POST"));

        /* Get posted field value */
        if (wifly.match(F("user="))) {
          wifly.gets(username, sizeof(username));
          wifly.flushRx();		// discard rest of input
          sendGreeting(username);
          //Serial.println(F("Sent greeting page"));
        }
      } 
      else {
        /* Unexpected request */
        //Serial.print(F("Unexpected: "));
        //Serial.println(buf);
        wifly.flushRx();		// discard rest of input
        //Serial.println(F("Sending 404"));
        send404();
      }
    }
  }

//  if (Serial.available() > 0) {
//    wifly.write(Serial.read());
//  }
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

/** Send an index HTML page with an input box for a username */
void sendIndex()
{
  /* Send the header direclty with print */
  wifly.println(F("HTTP/1.1 200 OK"));
  wifly.println(F("Content-Type: text/html"));
  wifly.println(F("Transfer-Encoding: chunked"));
  wifly.println();

  /* Send the body using the chunked protocol so the client knows when
   * the message is finished.
   * Note: we're not simply doing a close() because in version 2.32
   * firmware the close() does not work for client TCP streams.
   */
  wifly.sendChunkln(F("<html>"));
  wifly.sendChunkln(F("<title>WiFly HTTP Server Example</title>"));
  wifly.sendChunkln(F("<h1>"));
  wifly.sendChunkln(F("<p>Hello</p>"));
  wifly.sendChunkln(F("</h1>"));
  wifly.sendChunkln(F("<form name=\"input\" action=\"/\" method=\"post\">"));
  wifly.sendChunkln(F("Username:"));
  wifly.sendChunkln(F("<input type=\"text\" name=\"user\" />"));
  wifly.sendChunkln(F("<input type=\"submit\" value=\"Submit\" />"));
  wifly.sendChunkln(F("</form>")); 
  wifly.sendChunkln(F("</html>"));
  wifly.sendChunkln();
}

/** Send a greeting HTML page with the user's name and an analog reading */
void sendGreeting(char *name)
{
  /* Send the header directly with print */
  wifly.println(F("HTTP/1.1 200 OK"));
  wifly.println(F("Content-Type: text/html"));
  wifly.println(F("Transfer-Encoding: chunked"));
  wifly.println();

  /* Send the body using the chunked protocol so the client knows when
   * the message is finished.
   */
  wifly.sendChunkln(F("<html>"));
  wifly.sendChunkln(F("<title>WiFly HTTP Server Example</title>"));
  /* No newlines on the next parts */
  wifly.sendChunk(F("<h1><p>Hello "));
  wifly.sendChunk(name);
  /* Finish the paragraph and heading */
  wifly.sendChunkln(F("</p></h1>"));

  /* Include a reading from Analog pin 0 */
  snprintf_P(buf, sizeof(buf), PSTR("<p>Analog0=%d</p>"), analogRead(A0));
  wifly.sendChunkln(buf);

  wifly.sendChunkln(F("</html>"));
  wifly.sendChunkln();
}

/** Send a 404 error */
void send404()
{
  wifly.println(F("HTTP/1.1 404 Not Found"));
  wifly.println(F("Content-Type: text/html"));
  wifly.println(F("Transfer-Encoding: chunked"));
  wifly.println();
  wifly.sendChunkln(F("<html><head>"));
  wifly.sendChunkln(F("<title>404 Not Found</title>"));
  wifly.sendChunkln(F("</head><body>"));
  wifly.sendChunkln(F("<h1>Not Found</h1>"));
  wifly.sendChunkln(F("<hr>"));
  wifly.sendChunkln(F("</body></html>"));
  wifly.sendChunkln();
}








