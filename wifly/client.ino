// Wifly/Processing client
// Kevin Walker 14 Feb 2013
// Based on Example by Tom Igoe

import processing.net.*; 
Client myClient; 
String inString;
//byte interesting = 10;
int a = 0;

void setup() { 
  size (300, 100);
  myClient = new Client(this, "192.168.1.103", 2000);
  //myClient.write("$$$");
  background(255); 
  stroke(0);
} 

void draw() { 

  //GET THE DATA
  if (myClient.available() > 0) { 
    inString = myClient.readString(); 
    println(float(inString));

    //DRAW GRAPH
    a = a + 1;
    if (a > width) { 
      a = 0; 
      background(0);
    }
    point(a, float(inString));
  }
  
  //SAVE A DRAWING - UNCOMMMENT THESE
  saveFrame(); 

}

