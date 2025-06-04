// Broadcasts readings from all sensors on the Arduino Nano BLE Sense Rev 2 via Bluetooth
// Kevin Walker Jun 2025

#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <PDM.h>

BLEService contextService("180A"); // Custom service UUID

BLEFloatCharacteristic tempChar("2A6E", BLERead | BLENotify); // Temperature (float)
BLEFloatCharacteristic humChar("2A6F", BLERead | BLENotify);  // Humidity (float)
BLEFloatCharacteristic proxChar("2A19", BLERead | BLENotify); // Proximity (float, custom UUID)
BLEFloatCharacteristic accelXChar("2A77", BLERead | BLENotify); // Accel X
BLEFloatCharacteristic accelYChar("2A78", BLERead | BLENotify); // Accel Y
BLEFloatCharacteristic accelZChar("2A79", BLERead | BLENotify); // Accel Z
BLEFloatCharacteristic gyroXChar("2A80", BLERead | BLENotify);  // Gyro X
BLEFloatCharacteristic gyroYChar("2A81", BLERead | BLENotify);  // Gyro Y
BLEFloatCharacteristic gyroZChar("2A82", BLERead | BLENotify);  // Gyro Z
// Custom 128-bit UUIDs for magnetometer
BLEFloatCharacteristic magXChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic magYChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic magZChar("19B10007-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
// Custom 128-bit UUIDs for color
BLEFloatCharacteristic colorRedChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic colorGreenChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic colorBlueChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic colorClearChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic micLevelChar("2A9C", BLERead | BLENotify); // Microphone RMS
BLEFloatCharacteristic pressureChar("2A6D", BLERead | BLENotify); // Pressure (hPa)

// Microphone globals
volatile int samplesRead = 0;
short sampleBuffer[256];
float micLevel = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  Serial.println("HS300x started.");

  if (!APDS.begin()) {
    Serial.println("Failed to initialize proximity/color sensor!");
    while (1);
  }
  Serial.println("APDS9960 started.");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU started.");

  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  Serial.println("LPS22HB started.");

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) { // mono, 16kHz
    Serial.println("Failed to start PDM!");
    while (1);
  }
  Serial.println("PDM started.");

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
  contextService.addCharacteristic(proxChar);
  contextService.addCharacteristic(accelXChar);
  contextService.addCharacteristic(accelYChar);
  contextService.addCharacteristic(accelZChar);
  contextService.addCharacteristic(gyroXChar);
  contextService.addCharacteristic(gyroYChar);
  contextService.addCharacteristic(gyroZChar);
  contextService.addCharacteristic(magXChar);
  contextService.addCharacteristic(magYChar);
  contextService.addCharacteristic(magZChar);
  contextService.addCharacteristic(colorRedChar);
  contextService.addCharacteristic(colorGreenChar);
  contextService.addCharacteristic(colorBlueChar);
  contextService.addCharacteristic(colorClearChar);
  contextService.addCharacteristic(micLevelChar);
  contextService.addCharacteristic(pressureChar);
  BLE.addService(contextService);

  tempChar.writeValue(0.0);
  humChar.writeValue(0.0);
  proxChar.writeValue(0.0);
  accelXChar.writeValue(0.0);
  accelYChar.writeValue(0.0);
  accelZChar.writeValue(0.0);
  gyroXChar.writeValue(0.0);
  gyroYChar.writeValue(0.0);
  gyroZChar.writeValue(0.0);
  magXChar.writeValue(0.0);
  magYChar.writeValue(0.0);
  magZChar.writeValue(0.0);
  colorRedChar.writeValue(0.0);
  colorGreenChar.writeValue(0.0);
  colorBlueChar.writeValue(0.0);
  colorClearChar.writeValue(0.0);
  micLevelChar.writeValue(0.0);
  pressureChar.writeValue(0.0);

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
      float proximity = APDS.readProximity();
      float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
      float mx = 0, my = 0, mz = 0;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
      }
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
      }
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
      }
      int red = 0, green = 0, blue = 0, clear = 0;
      if (APDS.colorAvailable()) {
        APDS.readColor(red, green, blue, clear);
      }
      float pressure = BARO.readPressure();
      if (samplesRead) {
        long sum = 0;
        for (int i = 0; i < samplesRead; i++) {
          sum += sampleBuffer[i] * sampleBuffer[i];
        }
        micLevel = sqrt((float)sum / samplesRead);
        samplesRead = 0;
      }
      tempChar.writeValue(temperature);
      humChar.writeValue(humidity);
      proxChar.writeValue(proximity);
      accelXChar.writeValue(ax);
      accelYChar.writeValue(ay);
      accelZChar.writeValue(az);
      gyroXChar.writeValue(gx);
      gyroYChar.writeValue(gy);
      gyroZChar.writeValue(gz);
      magXChar.writeValue(mx);
      magYChar.writeValue(my);
      magZChar.writeValue(mz);
      colorRedChar.writeValue((float)red);
      colorGreenChar.writeValue((float)green);
      colorBlueChar.writeValue((float)blue);
      colorClearChar.writeValue((float)clear);
      micLevelChar.writeValue(micLevel);
      pressureChar.writeValue(pressure);

      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" C, Humidity: ");
      Serial.print(humidity);
      Serial.print(" %, Proximity: ");
      Serial.print(proximity);
      Serial.print(", Accel: ");
      Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.print(az);
      Serial.print(", Gyro: ");
      Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.print(gz);
      Serial.print(", Mag: ");
      Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.print(mz);
      Serial.print(", Color: R="); Serial.print(red);
      Serial.print(" G="); Serial.print(green);
      Serial.print(" B="); Serial.print(blue);
      Serial.print(" C="); Serial.print(clear);
      Serial.print(", Mic RMS: "); Serial.print(micLevel);
      Serial.print(", Pressure: "); Serial.println(pressure);

      delay(1000);
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
