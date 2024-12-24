// adxl335 accelerometer + XBee/serial
// by Kevin Walker 22 Mar 2015
// commissioned by National Football Museum 
// for RCA Space Program: Football & Physics project  

int saved = 0;
int sum = 0;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int x = analogRead(A0);
  int y = analogRead(A1);
  int z = analogRead(A2);
  sum = x+y+z;

  if (sum < saved-3) {
    Serial.print(sum);
    Serial.print(",");
  }
  if (sum > saved+3) {
    Serial.print(sum);
     Serial.print(",");
  }

  saved = sum;
  //Serial.println(sum);
  delay(10); // Minimum delay of 2 milliseconds between sensor reads (500 Hz)
}





