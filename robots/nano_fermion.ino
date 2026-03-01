// Fermion HR8833 Motor Driver Test 
// Arduino Nano 33 BLE Sense Rev1
// interactive test - uses serial & key presses 
// Kevin Walker 01 Mar 2026

// Motor 1 pins (D8/D7 swapped to match actual Fermion wiring)
const int M1A = 9;
const int M1B = 7;

// Motor 2 pins
const int M2A = 8;
const int M2B = 6;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection
  while (!Serial) {
    delay(10);
  }
  
  delay(1000);
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Fermion HR8833 Motor Driver Test");
  Serial.println("  Arduino Nano 33 BLE Sense Rev1");
  Serial.println("========================================");
  Serial.println();
  Serial.println("** SET SERIAL MONITOR TO 'NO LINE ENDING' **");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  1 = Motor 1 FORWARD");
  Serial.println("  2 = Motor 1 REVERSE");
  Serial.println("  3 = Motor 1 STOP");
  Serial.println();
  Serial.println("  4 = Motor 2 FORWARD");
  Serial.println("  5 = Motor 2 REVERSE");
  Serial.println("  6 = Motor 2 STOP");
  Serial.println();
  Serial.println("  f = Both motors FORWARD");
  Serial.println("  r = Both motors REVERSE");
  Serial.println("  s = STOP ALL motors");
  Serial.println();
  Serial.println("  p = PWM speed test");
  Serial.println("  h = Show this help");
  Serial.println();
  
  // Configure all pins as outputs
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  
  // Start with all motors stopped
  stopAll();
  
  Serial.println("All motors stopped");
  Serial.println("Ready! Press a key...");
  Serial.println();
}

void stopAll() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
}

void motor1Forward() {
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  Serial.println(">>> Motor 1: FORWARD");
}

void motor1Reverse() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  Serial.println(">>> Motor 1: REVERSE");
}

void motor1Stop() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  Serial.println(">>> Motor 1: STOP");
}

void motor2Forward() {
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  Serial.println(">>> Motor 2: FORWARD");
}

void motor2Reverse() {
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
  Serial.println(">>> Motor 2: REVERSE");
}

void motor2Stop() {
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
  Serial.println(">>> Motor 2: STOP");
}

void bothForward() {
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  Serial.println(">>> BOTH motors: FORWARD");
}

void bothReverse() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
  Serial.println(">>> BOTH motors: REVERSE");
}

void pwmSpeedTest() {
  // analogWrite breaks USB serial on the nRF52840, so this test
  // runs Motor 1 forward at full speed for 2 seconds instead.
  Serial.println();
  Serial.println("=== Motor 1 Run Test ===");
  Serial.println("Motor 1 forward full speed for 2 seconds...");
  motor1Forward();
  delay(2000);
  motor1Stop();
  Serial.println("Test complete");
  Serial.println();
}

void showHelp() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  Motor 1:");
  Serial.println("    1 = Forward");
  Serial.println("    2 = Reverse");
  Serial.println("    3 = Stop");
  Serial.println();
  Serial.println("  Motor 2:");
  Serial.println("    4 = Forward");
  Serial.println("    5 = Reverse");
  Serial.println("    6 = Stop");
  Serial.println();
  Serial.println("  Both Motors:");
  Serial.println("    f = Forward");
  Serial.println("    r = Reverse");
  Serial.println("    s = Stop all");
  Serial.println();
  Serial.println("  Other:");
  Serial.println("    p = PWM test");
  Serial.println("    h = Help");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Echo the command received (for debugging)
    Serial.print("Command received: '");
    Serial.print(cmd);
    Serial.print("' (ASCII ");
    Serial.print((int)cmd);
    Serial.println(")");
    
    switch(cmd) {
      // Motor 1 controls
      case '1':
        motor1Forward();
        break;
        
      case '2':
        motor1Reverse();
        break;
        
      case '3':
        motor1Stop();
        break;
      
      // Motor 2 controls
      case '4':
        motor2Forward();
        break;
        
      case '5':
        motor2Reverse();
        break;
        
      case '6':
        motor2Stop();
        break;
      
      // Both motors - lowercase
      case 'f':
        bothForward();
        break;
        
      case 'r':
        bothReverse();
        break;
        
      case 's':
        stopAll();
        Serial.println(">>> ALL STOP");
        break;
        
      // Both motors - uppercase
      case 'F':
        bothForward();
        break;
        
      case 'R':
        bothReverse();
        break;
        
      case 'S':
        stopAll();
        Serial.println(">>> ALL STOP");
        break;
      
      // PWM test - lowercase and uppercase
      case 'p':
      case 'P':
        pwmSpeedTest();
        break;
      
      // Help - lowercase and uppercase
      case 'h':
      case 'H':
        showHelp();
        break;
      
      // Ignore newlines and carriage returns
      case '\n':
      case '\r':
      case ' ':
        // Silently ignore
        break;
        
      default:
        Serial.print(">>> Unknown command: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println(">>> Press 'h' for help");
        break;
    }
  }
}
