
#include <ArduinoBLE.h>
#include "BluetoothPeripheral.hpp"
#include "MotionHandler.hpp"
#include "RgbLed.hpp"
//#include "IRControl.hpp" // for rp2040 connect
#include "IRControlBleSense.hpp" // for ble sense

BluetoothPeripheral btPeripheral(
    "1c0e6984-77ac-4a2c-88d0-0331c44c9b32",  // Service UUID
    "eda7f160-c43f-453e-bdbd-cbae7b01d49b",  // Characteristic UUID
    "NanoBLE"                              // Device Name
);

const int BUTTON_PIN = 3;

//IRControl irControl(25, 3); // for rp2040 connect
IrControlBleSense irControlBleSense(2); // for ble sense

RgbLed rgbLed(14, 16, 20);

MotionHandler motionHandler;

void setup() {
    Serial.begin(9600);
    if (!btPeripheral.begin()) {
        while (1);
    }
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    motionHandler.init();
    //irControl.init();
    irControlBleSense.init();
}

void loop() {
    long previousMillis = millis(); 
    const long interval = 1000;     
    while (!btPeripheral.isConnectedToCentral()) {
        rgbLed.blinkRed(200);
        if (millis() - previousMillis >= interval) {
            previousMillis = millis();
            Serial.println("Waiting for connection...");
        }
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        rgbLed.blinkYellow(100);
        irControlBleSense.update();
        btPeripheral.updateValue(4);
        btPeripheral.pool();
        if (btPeripheral.ifCharacteristicWritten()) {
            uint8_t value = btPeripheral.readValue();
            if (value == 2) {
                rgbLed.turnOnYellow();
                rgbLed.turnOnRed();
                Serial.println("Starting motion stream...");
                while (1);
            }
        } else {
            //rgbLed.blinkYellow(200);
            rgbLed.blinkRed(200);
        }
    } else {
        rgbLed.turnOff();
    }

    //irControl.update();
    //irControlBleSense.update();
    //String motion = motionHandler.processMotion();
    //if (motion != "") {
        //Serial.println(motion);
    //}
    //delay(20);
}
