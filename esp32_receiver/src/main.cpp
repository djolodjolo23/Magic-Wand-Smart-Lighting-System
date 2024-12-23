#include <Arduino.h>
#include "BluetoothCentral.hpp"
#include "IRReceive.hpp"

static const char* sUUID = "1c0e6984-77ac-4a2c-88d0-0331c44c9b32";
static const char* cUUID = "eda7f160-c43f-453e-bdbd-cbae7b01d49b";

const u_int8_t IR_RECEIVER_PIN_ONE = 32;
const u_int8_t IR_RECEIVER_PIN_TWO = 34;
const u_int8_t IR_RECEIVER_PIN_THREE = 35;
const u_int8_t GREEN_LED_PIN = 33;
const u_int8_t YELLOW_LED_PIN = 12;

BluetoothCentral btCentral(sUUID, cUUID, "ESP32_Central");
IRReceive irReceiver(IR_RECEIVER_PIN_ONE, IR_RECEIVER_PIN_TWO, IR_RECEIVER_PIN_THREE, GREEN_LED_PIN, YELLOW_LED_PIN);

void setup() {
  Serial.begin(9600);
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

  uint8_t biggestVal = irReceiver.listenForIr();
  if (biggestVal != 107) { // 107 is the unsuccesful ack value
    btCentral.writeValue(biggestVal);
    while (true) {
      uint8_t val = btCentral.readNewValue();
      if (val != 0) {
        Serial.print("New Motion Value:");
        Serial.println(val);
      }
      if (val == 105) { // 105 is the stop motion ack value
        break;
      }
      delay(20);  
    }
    
  }
  delay(200);
}
