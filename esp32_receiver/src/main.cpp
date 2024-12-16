#include <Arduino.h>
#include "BluetoothCentral.hpp"
#include "IRReceive.hpp"

static const char* sUUID = "18cfb27c-df3d-41c7-801b-60165e9c9872";
static const char* cUUID = "78eb176c-cef1-4295-8a5f-69923ca804b9";

BluetoothCentral btCentral(sUUID, cUUID, "ESP32_Central");
IRReceive irReceiver(32, 33);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  //BLEDevice::init("ESP32_Central");
}

void loop() {
  
  /*
  unsigned long previousMillis = millis();
  const long interval = 1000;
  while (!btCentral.isConnected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Attempting to connect...");
      btCentral.connect();
      previousMillis = currentMillis;
    }
  }
  Serial.println("Connected!");
  //uint8_t val = btCentral.readValue();
  //Serial.println(val);
  */
  irReceiver.listenForIR();
  
}
