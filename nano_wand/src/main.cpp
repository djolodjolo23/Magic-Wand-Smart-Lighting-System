
#include <ArduinoBLE.h>
#include "BluetoothPeripheral.hpp"
#include "MotionHandler.hpp"
#include "Leds.hpp"
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

Leds leds(14, 16, 18);

MotionHandler motionHandler;

void setup() {
    Serial.begin(9600);
    if (!btPeripheral.begin()) {
        while (1);
    }
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    motionHandler.init();
    //irControl.init(); // for rp2040 connect
    irControlBleSense.init(); // for ble sense
}

void loop() {
    long previousMillis = millis(); 
    const long interval = 1000;     
    while (!btPeripheral.isConnectedToCentral()) {
        leds.blinkRed(200);
        if (millis() - previousMillis >= interval) {
            previousMillis = millis();
            Serial.println("Waiting for connection...");
        }
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        leds.blinkAllTogether(200);
        irControlBleSense.update();
        btPeripheral.pool();
        if (btPeripheral.ifCharacteristicWritten()) {
            uint8_t value = btPeripheral.readValue();
            if (value == 106) {
                Serial.println("Starting motion stream...");
                while (true) {
                    leds.blinkAllSimultaneously(200);
                    uint8_t currentMotion = motionHandler.processMotion();
                    if (currentMotion > 0) {
                        Serial.println(currentMotion);
                        btPeripheral.updateValue(currentMotion);
                    }
                    //btPeripheral.updateValue(currentMotion);
                    if (digitalRead(BUTTON_PIN) == HIGH) {
                        leds.turnOff();
                        btPeripheral.updateValue(105); // 105 for end of motion stream
                        //delay(2000);
                        break;
                    }
                }
            }
        } else {
            //rgbLed.blinkYellow(200);
            //rgbLed.blinkRed(200);
        }
    } else {
        leds.turnOff();
        btPeripheral.updateValue(105); // 105 for end of motion stream
    }

    //irControl.update();
    //irControlBleSense.update();
    //String motion = motionHandler.processMotion();
    //if (motion != "") {
        //Serial.println(motion);
    //}
    //delay(20);
}
