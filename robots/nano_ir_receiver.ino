// IR receiver for simple 940nm LEDs (e.g. https://www.hellasdigital.gr/electronics/sensors/infrared-sensors/940nm-ir-infrared-receiver-led/
// Kevin Walker 10 Mar 2026

const int IR_PIN = A0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Raw IR phototransistor test");
  Serial.println("Lower number = more IR light");
  Serial.println();
}

void loop() {
  int raw = analogRead(IR_PIN);
  Serial.println(raw);
  delay(100);
}
