#include <Arduino.h>
#include "BluetoothCentral.hpp"

static const char* sUUID = "18cfb27c-df3d-41c7-801b-60165e9c9872";
static const char* cUUID = "78eb176c-cef1-4295-8a5f-69923ca804b9";

BluetoothCentral btCentral(sUUID, cUUID, "ESP32_Central");

void setup() {
  Serial.begin(9600);
  BLEDevice::init("ESP32_Central");
}

void loop() {
  if (!btCentral.isConnected()) {
    btCentral.connect();
    delay(1000);
  } else {
    uint8_t value = btCentral.readValue();
    Serial.print("Received: ");
    Serial.println(value);
  }
}
