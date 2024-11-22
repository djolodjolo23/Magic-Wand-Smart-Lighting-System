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
        uint8_t value;

    public:
        BluetoothPeripheral(const char* sUUID, const char* cUUID, const char* dName, uint8_t initialValue)
        : serviceUUID(sUUID),
          charUUID(cUUID),
          deviceName(dName),
          service(sUUID),
          characteristic(cUUID, BLERead | BLEWrite),
          value(initialValue)
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
            characteristic.writeValue(value);
            BLE.advertise();
            return true;
        }

        void updateValue() {
            characteristic.writeValue(value);
        }

        void setValue(uint8_t newValue) {
            value = newValue;
        }

        void handleConnection() {
            BLEDevice central = BLE.central();
            if (central) {
                Serial.print("Connected to central: ");
                Serial.println(central.address());
                
                while (central.connected()) {
                    updateValue();
                    Serial.print("Sent value: ");
                    Serial.println(value);
                    value = (value % 10) + 1; // Cycle through 1-10 for testing
                    delay(1000);
                }

                Serial.println("Disconnected from central.");
            } else {
                Serial.println("Not connected...");
                delay(1000);
            }
        }
};
