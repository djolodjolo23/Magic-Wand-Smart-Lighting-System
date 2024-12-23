#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>

class BluetoothPeripheral {

    private:
        const char* serviceUUID;
        const char* charUUID;
        const char* deviceName;
        BLEService service;
        BLEUnsignedCharCharacteristic characteristic;

    public:
        BluetoothPeripheral(const char* sUUID, const char* cUUID, const char* dName)
        : serviceUUID(sUUID),
          charUUID(cUUID),
          deviceName(dName),
          service(sUUID),
          characteristic(cUUID, BLERead | BLEWrite | BLENotify)
        {}

        bool begin() {
            if (!BLE.begin()) {
                Serial.println("starting BLE failed!");
                return false;
            }
            BLE.setLocalName(deviceName);
            BLE.setAdvertisedService(service);
            service.addCharacteristic(characteristic);
            BLE.addService(service);
            BLE.advertise();
            return true;
        }

        void pool() {
            BLE.poll();
        }

        void updateValue(uint8_t value) {
            characteristic.writeValue(value);
        }

        bool ifCharacteristicWritten() {
            return characteristic.written();
        }


        bool isConnectedToCentral() {
            return BLE.connected();
        }

        uint8_t readValue() {
            return characteristic.value();
        }
};
