
#include <ArduinoBLE.h>
#include "BluetoothPeripheral.hpp"
#include <MotionHandler.hpp>

uint8_t value = 0;

BluetoothPeripheral btPeripheral(
    "18cfb27c-df3d-41c7-801b-60165e9c9872",  // Service UUID
    "78eb176c-cef1-4295-8a5f-69923ca804b9",  // Characteristic UUID
    "NanoBLE"                              // Device Name
);

MotionHandler motionHandler;

void setup() {
    Serial.begin(115200);
    if (!btPeripheral.begin()) {
        while (1);
    }
    //motionHandler.init();
}

void loop() {
    //btPeripheral.handleConnection();
    //btPeripheral.updateValue(value);

    //Serial.print("Updating value to: ");
    //Serial.println(value);

    //value = (value % 10) + 1;
    //delay(1000);

    //String motion = motionHandler.processMotion();
    //if (motion != "") {
        //Serial.println(motion);
    //}
    //delay(20);
}
