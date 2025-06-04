// Arduino Nano BLE Sense Rev 2 broadcasts temperature and humidity via Bluetooth
// Kevin Walker June 2025

#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>

BLEService contextService("180A"); // Custom service UUID
BLEFloatCharacteristic tempChar("2A6E", BLERead | BLENotify); // Temperature characteristic (float)
BLEFloatCharacteristic humChar("2A6F", BLERead | BLENotify);  // Humidity characteristic (float)

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  Serial.println("HS300x started.");

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }
  Serial.println("BLE started.");

  BLE.setLocalName("ContextBeacon");
  BLE.setDeviceName("ContextBeacon");
  BLE.setAdvertisedService(contextService);

  contextService.addCharacteristic(tempChar);
  contextService.addCharacteristic(humChar);
  BLE.addService(contextService);

  tempChar.writeValue(0.0);
  humChar.writeValue(0.0);

  BLE.advertise();
  Serial.println("BLE ContextBeacon Peripheral");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      float temperature = HS300x.readTemperature();
      float humidity = HS300x.readHumidity();
      tempChar.writeValue(temperature);
      humChar.writeValue(humidity);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      delay(1000);
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
