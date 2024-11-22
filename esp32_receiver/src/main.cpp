#include <Arduino.h>
#include <BLEDevice.h>

static BLEUUID serviceUUID("18cfb27c-df3d-41c7-801b-60165e9c9872");
static BLEUUID charUUID("78eb176c-cef1-4295-8a5f-69923ca804b9");

BLEClient* client;
BLERemoteCharacteristic* remoteChar;


void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32_Central");
  client = BLEDevice::createClient();
}

void loop() {
  if (!client->isConnected()) {
    Serial.println("Not connected, scanning for devices..."); // attempt to connect to NanoBLE
    BLEScan* scan = BLEDevice::getScan();
    scan->setActiveScan(true); // active scan uses more power, but get results faster
    BLEScanResults results = scan->start(5);

    bool foundDevice = false;

    for (int i = 0; i < results.getCount(); i++) {
      BLEAdvertisedDevice device = results.getDevice(i);
      Serial.print("Found Device: ");
      Serial.println(device.getName().c_str());

      if (device.getName() == "NanoBLE") { // NanoBLE is the name of the peripheral device, if found, connect to it
        Serial.println("Found NanoBLE, attempting to connect...");
        client->connect(&device);
        foundDevice = true;
        break;
      }
    }

    if (foundDevice && client->isConnected()) {
      Serial.println("Connected to NanoBLE");

      BLERemoteService* remoteService = client->getService(serviceUUID);
      if (remoteService == nullptr) {
        Serial.println("Failed to find our service UUID");
        client->disconnect();
        return;
      }

      remoteChar = remoteService->getCharacteristic(charUUID);
      if (remoteChar == nullptr) {
        Serial.println("Failed to find our characteristic UUID");
        client->disconnect();
        return;
      }
      Serial.println("Found characteristic");
    } else {
      Serial.println("Failed to connect to NanoBLE");
    }

    scan->clearResults(); 
    delay(1000);

  } else {
    if (remoteChar && remoteChar->canRead()) {
      uint8_t value = remoteChar->readUInt8();
      Serial.print("Received: ");
      Serial.println(value);
    } else {
      Serial.println("Cannot read from characteristic");
    }

    if (!client->isConnected()) {
      Serial.println("Disconnected from NanoBLE");
    }
  }
}