#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "IRReceive.hpp"
#include "LedStrip.hpp"

const char* ssid = "Tele2_108703";
const char* password = "q2yymgzk";
const char* mqtt_server = "192.168.0.24";

const String clientId = "2";
const String motionSubTopic = "app/motions_2";
const String irPubTopic = "app/ir_read";

volatile bool shouldBlock = false;
volatile bool unblock = false;

WiFiClient espClient;
PubSubClient client(espClient);
IRReceive irReceiver(35);
LedStrip ledStrip(25, 3, 50);

void callBack(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    int val = message.toInt();

    Serial.println(val);

    if (val != 0 && val != 105) {
        ledStrip.testFunc(val);
        shouldBlock = true; // Enter blocking mode
    }

    if (val == 105) {
        unblock = true; // Set unblock flag
        shouldBlock = false;
    }
}


void setup() {
    Serial.begin(9600);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }

    while (!client.connected()) {
        client.setServer(mqtt_server, 1883);
        client.setCallback(callBack);
        if (client.connect(clientId.c_str())) {
            client.subscribe(motionSubTopic.c_str());
        }
    }
    Serial.println("Connected to WiFi");
    client.setServer(mqtt_server, 1883);
}

void loop() {
    if (!shouldBlock) {
        uint8_t val = irReceiver.listenForIr();
        if (val != 0) {
            client.publish(irPubTopic.c_str(), (clientId + ":" + String(val)).c_str());
        }
    } else {
        while (!unblock) {
            client.loop(); 
            delay(20);
        }
        unblock = false; 
    }

    client.loop();
    delay(200);
}