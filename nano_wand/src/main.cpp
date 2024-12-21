
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

//IRControl irControl(25, 3); // for rp2040 connect

IrControlBleSense irControlBleSense(2, 3); // for ble sense

RgbLed rgbLed(14, 19, 20);

MotionHandler motionHandler;

void setup() {
    Serial.begin(9600);
    if (!btPeripheral.begin()) {
        while (1);
    }
    motionHandler.init();
    //irControl.init();
    irControlBleSense.init();
}

void loop() {
    long startTime = millis();
    long previousMillis = millis(); 
    const long interval = 1000;     
    while (!btPeripheral.isConnectedToCentral()) {
        rgbLed.blinkRed(150);
        if (millis() - previousMillis >= interval) {
            previousMillis = millis();
            //rgbLed.turnOnRed();
            rgbLed.blinkRedNonBlocking(startTime, 150);
            Serial.println("Waiting for connection...");
        }
        //rgbLed.turnOff();
    }

    //irControl.update();
    irControlBleSense.update();

    //String motion = motionHandler.processMotion();
    //if (motion != "") {
        //Serial.println(motion);
    //}
    //delay(20);
}
