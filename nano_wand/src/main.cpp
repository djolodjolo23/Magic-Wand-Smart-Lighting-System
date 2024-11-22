#include <Arduino.h>
#include <ArduinoBLE.h>

BLEService numberService("18cfb27c-df3d-41c7-801b-60165e9c9872");
BLEUnsignedCharCharacteristic numberCharacteristic("78eb176c-cef1-4295-8a5f-69923ca804b9", BLERead | BLEWrite);

u_int8_t num = 1;

void setup() {

  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("NanoBLE");
  BLE.setAdvertisedService(numberService);
  numberService.addCharacteristic(numberCharacteristic);
  BLE.addService(numberService);

  numberCharacteristic.writeValue(num);

  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

void loop() {
  BLEDevice central = BLE.central(); // nano is peripheral since transmitting, esp32 is central, find the central device
  if (central) { // if central device is found
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()) {
      numberCharacteristic.writeValue(num);
      num = (num % 10) + 1; // for now I just want to cycle through 1-10 for testing
      delay(1000);
    }
    Serial.print("Disconnected from central: ");
  }
  Serial.println("Not connected...");
  delay(1000);
}

