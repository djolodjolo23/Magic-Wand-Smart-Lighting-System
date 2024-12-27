#include <Arduino.h>
#include "BluetoothCentral.hpp"
#include "IRReceive.hpp"
#include "LedStrip.hpp"

static const char* sUUID = "1c0e6984-77ac-4a2c-88d0-0331c44c9b32";
static const char* cUUID = "eda7f160-c43f-453e-bdbd-cbae7b01d49b";

const u_int8_t IR_RECEIVER_PIN_ONE = 32;
const u_int8_t IR_RECEIVER_PIN_TWO = 35;
const u_int8_t IR_RECEIVER_PIN_THREE = 34;

const u_int8_t LED_STRIP_PIN_ONE = 26; // 12 is good
const u_int8_t LED_STRIP_PIN_TWO = 25;
const u_int8_t LED_STRIP_PIN_THREE = 33;
const u_int8_t LED_STRIP_COUNT = 5;
const u_int8_t LED_STRIP_BRIGHTNESS = 50;

BluetoothCentral btCentral(sUUID, cUUID, "ESP32_Central");
IRReceive irReceiver(IR_RECEIVER_PIN_ONE, IR_RECEIVER_PIN_TWO, IR_RECEIVER_PIN_THREE);
LedStrip ledStripOne(LED_STRIP_PIN_ONE, LED_STRIP_COUNT, LED_STRIP_BRIGHTNESS);
LedStrip ledStripTwo(LED_STRIP_PIN_TWO, LED_STRIP_COUNT, LED_STRIP_BRIGHTNESS);
LedStrip ledStripThree(LED_STRIP_PIN_THREE, LED_STRIP_COUNT, LED_STRIP_BRIGHTNESS);

void setup() {
  Serial.begin(9600);
  BLEDevice::init("ESP32_Central");
  ledStripOne.beginAndShow();
  ledStripTwo.beginAndShow();
  ledStripThree.beginAndShow();
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
  Serial.print("Biggest val: ");
  Serial.println(biggestVal);
  if (biggestVal != 109) { // 109 is the unsuccesful ack value
    btCentral.writeValue(biggestVal);
    while (true) {
      uint8_t val = btCentral.readNewValue();
      if (val != 0) {
        Serial.print("Received value: ");
        Serial.println(val);
        if (biggestVal == 106) {
          ledStripOne.testFunc(val);
        } else if (biggestVal == 107) {
          ledStripTwo.testFunc(val);
        } else if (biggestVal == 108) {
          ledStripThree.testFunc(val);
        }
      }
      if (val == 105) { // 105 is the stop motion ack value
        break;
      }
      delay(20);  
    }
    
  }
  delay(200);
}
