#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>

class BluetoothPeripheral {

    private:
        BLEService service;
        BLEUnsignedCharCharacteristic characteristic;
        u_int8_t value;

    public:
        BluetoothPeripheral(const char* serviceUUID, const char* charUUID, const char* deviceName, uint8_t initialValue)
        : service(serviceUUID), 
          characteristic(charUUID, BLERead | BLEWrite), 
          value(initialValue) 
    {
        BLE.setLocalName(deviceName);
        BLE.setAdvertisedService(service);
        service.addCharacteristic(characteristic);
        BLE.addService(service);
        characteristic.writeValue(value);
    }

    bool begin() {
        if (!BLE.begin()) {
            return false;
        }
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
                value = (value % 10) + 1; // for now I just want to cycle through 1-10 for testing
                delay(1000);
            }

            Serial.println("Disconnected from central.");
        } else {
            Serial.println("Not connected...");
            delay(1000);
        }
    }


};