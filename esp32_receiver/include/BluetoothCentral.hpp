#pragma once

#include <Arduino.h>
#include <BLEDevice.h>

class BluetoothCentral {
    private:
        const char* serviceUUID;
        const char* charUUID;
        BLEClient* client;
        BLERemoteCharacteristic* remoteChar;
};