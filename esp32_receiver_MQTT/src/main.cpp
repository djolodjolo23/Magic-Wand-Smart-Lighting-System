#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "IRReceive.hpp"
#include "LedStrip.hpp"

const char* ssid = "Tele2_108703";
const char* password = "q2yymgzk";
const char* mqtt_server = "192.168.0.31";

const String clientId = "1";
//const String clientId = "2";
//const String clientId = "3";

const String motionSubTopic = "app/motions_1";
//const String motionSubTopic = "app/motions_2";
//const String motionSubTopic = "app/motions_3";
const String irPubTopic = "app/ir_read";

volatile bool shouldBlock = false;
volatile bool unblock = false;

WiFiClient espClient;
PubSubClient client(espClient);
IRReceive irReceiver(5);
LedStrip ledStrip(4, 3, 50);

// Global variables to store topic and message
String incomingTopic = "";
String incomingMessage = "";

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    incomingTopic = topic;  // Store the topic
    incomingMessage = "";   // Clear the previous message
    for (unsigned int i = 0; i < length; i++) {
        incomingMessage += (char)payload[i]; // Store the payload
    }
}

void processMessage(String topic, String message) {
    int val = message.toInt();

    Serial.println("Processing Message:");
    Serial.println("Topic: " + topic);
    Serial.println("Message: " + message);

    if (val != 0 && val != 105) {
        ledStrip.testFunc(val);
        shouldBlock = true; // Enter blocking mode
    }

    if (val == 105) {
        unblock = true; // Set unblock flag
        shouldBlock = false;
    }
}

void connectToWiFi() {
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nWi-Fi connected");
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT broker");
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback); // Keep callback for message buffering
    while (!client.connected()) {
        Serial.print(".");
        if (client.connect(clientId.c_str())) {
            client.subscribe(motionSubTopic.c_str());
            Serial.println("\nMQTT connected and subscribed to: " + motionSubTopic);
        } else {
            delay(1000);
        }
    }
}

void checkMQTTMessages() {
    client.loop(); // Ensure MQTT messages are processed

    // Process the buffered message
    if (incomingTopic != "" && incomingMessage != "") {
        processMessage(incomingTopic, incomingMessage);

        // Clear the buffers after processing
        incomingTopic = "";
        incomingMessage = "";
    }
}

void setup() {
    Serial.begin(9600);

    connectToWiFi();
    connectToMQTT();
}

void loop() {
    // Process MQTT messages in the main loop
    checkMQTTMessages();

    // Handle IR receiver when not blocking
    if (!shouldBlock) {
        uint8_t val = irReceiver.listenForIrUpdated();
        Serial.println(val);
        if (val != 0) {
            client.publish(irPubTopic.c_str(), (clientId + ":" + String(val)).c_str());
        }
    } else {
        // Handle unblock mechanism
        while (!unblock) {
            checkMQTTMessages(); // Process MQTT messages during blocking
            delay(20);
        }
        unblock = false;
    }
}
