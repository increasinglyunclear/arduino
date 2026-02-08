// Control 2 motors using Arduino Nano BLE Sense Rev 2 and L293DNE motor control IC.
// Also works with L298N motor controller

// connections:
// +5V → Arduino VIN, L293DNE Pins 1, 8, 9, 16 (from laptop or USB charger, or 7.4V LiPo battery)
// GND → Arduino GND, L293DNE Pins 4, 5, 12, 13
// D9 → L293DNE Pin 2
// D8 → L293DNE Pin 7
// D7 → L293DNE Pin 10
// D6 → L293DNE Pin 15
// Motor A to L293DNE Pins 3, 6
// Motor B to L293DNE Pins 11, 14

// L293DNE motor control pins declaration:
// Motor A
int in1 = 9;
int in2 = 8;
// Motor B
int in3 = 7;
int in4 = 6;

void setup() {
  Serial.begin(9600);
  Serial.println("L293DNE Motor Test Starting...");
  
  // Set all motor control pins to outputs:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  Serial.println("Pins configured. Starting demo in 2 seconds...");
  delay(2000);
}

void demoOne() {
  Serial.println("Demo One: Forward/Reverse (full speed)");
  
  // Motor A forward, Motor B forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  Serial.println("Motors forward");
  delay(2000);
  
  // Motor A reverse, Motor B reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  Serial.println("Motors reverse");
  delay(2000);
  
  // Stop motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  Serial.println("Motors stopped");
}

void loop() {
  demoOne();
  delay(1000);
}
