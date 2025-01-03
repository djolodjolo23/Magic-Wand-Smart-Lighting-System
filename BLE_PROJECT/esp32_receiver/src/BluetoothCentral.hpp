#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

class BluetoothCentral {
private:
    BLEUUID serviceUUID;
    BLEUUID charUUID;
    BLEClient* client;
    BLERemoteCharacteristic* remoteChar;
    String targetDeviceName;

    static BluetoothCentral* instance;
    bool newValueAvailable;
    uint8_t lastReadValue;

    static void notifyCallBack(
        BLERemoteCharacteristic *characteristic,
        uint8_t *data,
        size_t length,
        bool isNotify
    ) {
        if (instance && length > 0) {
            instance->lastReadValue = data[0];
            instance->newValueAvailable = true;
        }
    }

public:
    BluetoothCentral(const char* sUUID, const char* cUUID, const char* deviceName)
    : serviceUUID(BLEUUID(sUUID)),
      charUUID(BLEUUID(cUUID)),
      targetDeviceName(deviceName),
      client(nullptr),
      remoteChar(nullptr),
      newValueAvailable(false),
      lastReadValue(0)
    {
        client = BLEDevice::createClient();
        instance = this;
    }

    bool connect() {
        if (client->isConnected()) {
            Serial.println("Already connected.");
            return true;
        }

        Serial.println("Not connected, scanning for devices...");
        BLEScan* scan = BLEDevice::getScan();
        scan->setActiveScan(true); // active scan
        BLEScanResults results = scan->start(5); // scan for 5 seconds

        bool foundDevice = false;
        BLEAdvertisedDevice targetAdvDevice;

        for (int i = 0; i < results.getCount(); i++) {
            BLEAdvertisedDevice device = results.getDevice(i);
            String devName = device.getName().c_str();
            Serial.print("Found device: ");
            Serial.println(devName);

            if (devName == "NanoBLE") {
                Serial.println("Found NanoBLE, attempting to connect...");
                targetAdvDevice = device;
                foundDevice = true;
                break;
            }
        }

        scan->clearResults();

        if (!foundDevice) {
            Serial.println("Failed to find NanoBLE.");
            return false;
        }

        if (client->connect(&targetAdvDevice)) {
            Serial.println("Connected to NanoBLE.");

            BLERemoteService* remoteService = client->getService(serviceUUID);
            if (remoteService == nullptr) {
                Serial.println("Failed to find our service UUID.");
                client->disconnect();
                return false;
            }

            remoteChar = remoteService->getCharacteristic(charUUID);
            if (remoteChar == nullptr) {
                Serial.println("Failed to find our characteristic UUID.");
                client->disconnect();
                return false;
            }

            Serial.println("Found characteristic successfully.");
            if (remoteChar -> canNotify()) {
                remoteChar->registerForNotify(notifyCallBack);
                Serial.println("Registered for notifications.");
            }
            return true;
        } else {
            Serial.println("Failed to connect to NanoBLE.");
            return false;
        }
    }

    uint8_t readNewValue() {
        if (newValueAvailable) {
            newValueAvailable = false; // Reset the flag
            return lastReadValue;      // Return the new value
        }
        return 0; // No new value available
    }


    void disconnect() {
        if (client && client->isConnected()) {
            client->disconnect();
            Serial.println("Disconnected from NanoBLE.");
        }
    }

    bool isConnected() {
        return client && client->isConnected();
    }

    uint8_t readValue() {
        if (remoteChar && remoteChar->canRead()) {
            return remoteChar->readUInt8();
        }
        return 0;
    }


    bool writeValue(uint8_t val) {
        if (remoteChar && remoteChar->canWrite()) {
            remoteChar->writeValue(val, 1);
            Serial.print("Wrote value: ");
            Serial.println(val);
            return true;
        }
        Serial.println("Characteristic not writable or not found.");
        return false;
    }
};

BluetoothCentral* BluetoothCentral::instance = nullptr;
