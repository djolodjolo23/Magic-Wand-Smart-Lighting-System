#include <Arduino.h>
#include "BluetoothCentral.hpp"
#include "IRReceive.hpp"

static const char* sUUID = "1c0e6984-77ac-4a2c-88d0-0331c44c9b32";
static const char* cUUID = "eda7f160-c43f-453e-bdbd-cbae7b01d49b";

BluetoothCentral btCentral(sUUID, cUUID, "ESP32_Central");
IRReceive irReceiver(32, 34, 35, 33, 12);

void setup() {
  Serial.begin(9600);
  //Serial.println("Starting...");
  BLEDevice::init("ESP32_Central");
}

void loop() {
  unsigned long previousMillis = millis();
  const long bleConnectionInterval = 300;
  while (!btCentral.isConnected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= bleConnectionInterval) {
      Serial.println("Attempting to connect...");
      btCentral.connect();
      previousMillis = currentMillis;
    }
  }

  //irReceiver.listenForIR();
  uint8_t biggestVal = irReceiver.testFunc();
  Serial.println(biggestVal);
  btCentral.writeValue(biggestVal);
  //delay(20);
  delay(200);
  
}
