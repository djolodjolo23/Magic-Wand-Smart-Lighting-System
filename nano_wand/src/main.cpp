
#include <ArduinoBLE.h>
#include "BluetoothPeripheral.hpp"
#include "MotionHandler.hpp"
#include "RgbLed.hpp"
#include "IRControl.hpp"

BluetoothPeripheral btPeripheral(
    "1c0e6984-77ac-4a2c-88d0-0331c44c9b32",  // Service UUID
    "eda7f160-c43f-453e-bdbd-cbae7b01d49b",  // Characteristic UUID
    "NanoBLE"                              // Device Name
);

int value = 0;

IRControl irControl(25, 3);
RgbLed rgbLed(16, 19, 20);
MotionHandler motionHandler;

void setup() {
    Serial.begin(115200);
    if (!btPeripheral.begin()) {
        while (1);
    }
    //motionHandler.init();
    irControl.init();
}

void loop() {
    long previousMillis = millis(); 
    const long interval = 1000;     
    while (!btPeripheral.isConnectedToCentral()) {
        if (millis() - previousMillis >= interval) {
            previousMillis = millis();
            //rgbLed.turnOnRed();
            Serial.println("Waiting for connection...");
        }
        //rgbLed.turnOff();
    }
    btPeripheral.updateValue(value);

    Serial.print("Updating value to: ");
    Serial.println(value);

    value = (value % 10) + 1;
    delay(1000);

    //irControl.update();

    //String motion = motionHandler.processMotion();
    //if (motion != "") {
        //Serial.println(motion);
    //}
    //delay(20);
}
