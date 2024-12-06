#pragma once

#include <Arduino.h>
#include <BLEDevice.h>

class BluetoothCentral {
    private:
        const char* serviceUUID;
        const char* charUUID;
        BLEClient* client;
        BLERemoteCharacteristic* remoteChar;

    public:
        BluetoothCentral(const char* sUUID, const char* cUUID, const char* deviceName)
        : serviceUUID(sUUID),
          charUUID(cUUID)
        {
            BLEDevice::init(deviceName);
            client = BLEDevice::createClient();
        }

        bool connect() {
            if (client->isConnected()) {
                Serial.println("Already connected to NanoBLE.");
                return true;
            }

            Serial.println("Not connected, scanning for devices...");
            BLEScan* scan = BLEDevice::getScan();
            scan->setActiveScan(true);
            BLEScanResults results = scan->start(5, false); // Scan for 5 seconds

            bool foundDevice = false;

            for (int i = 0; i < results.getCount(); i++) {
                BLEAdvertisedDevice device = results.getDevice(i);
                const char* deviceName = device.getName().c_str();

                if (deviceName && String(deviceName) == "NanoBLE") {
                    Serial.println("Found NanoBLE, attempting to connect...");
                    if (client->connect(&device)) {
                        foundDevice = true;
                        break;
                    } else {
                        Serial.println("Failed to connect to device.");
                    }
                }
            }

            scan->clearResults();

            if (foundDevice && client->isConnected()) {
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
                return true;
            } else {
                Serial.println("Failed to find or connect to NanoBLE.");
                return false;
            }
        }

        void disconnect() {
            if (client->isConnected()) {
                client->disconnect();
                Serial.println("Disconnected from NanoBLE.");
            }
        }

        bool isConnected() {
            return client->isConnected();
        }
};
