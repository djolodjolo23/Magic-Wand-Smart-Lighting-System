#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DSOX.h> // for rp2040 connect
#include "MotionHandler.hpp"
#include "Leds.hpp"
#include "IRControl.hpp"

const char* ssid = "Pixel_4585";
const char* password = "123456789";

const char* mqtt_server = "192.168.0.24";
const char* mqtt_client_id = "NanoClient";  // Unique ID for this device
const char* mqtt_sub_topic = "app/ir_read";
const char* mqtt_pub_topic = "app/motions_2";

const int BUTTON_PIN = 3;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
MotionHandler motionHandler;
IRControl irControl(25);
Leds leds(14, 16, 18);

int client1IrValue = 0;
int client2IrValue = 0;
int client3IrValue = 0;

// -----------------------------------------------------------------------------
// MQTT Callback
// -----------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message: ");
    Serial.println(message);

    // Parse "clientID:value"
    int separatorIndex = message.indexOf(':');
    if (separatorIndex != -1) {
        int clientID = message.substring(0, separatorIndex).toInt();
        int value    = message.substring(separatorIndex + 1).toInt();

        if (clientID == 1) {
            client1IrValue = value;
        } else if (clientID == 2) {
            client2IrValue = value;
        } else if (clientID == 3) {
            client3IrValue = value;
        }
    }

    if (client1IrValue >= client2IrValue && client1IrValue >= client3IrValue) {
        Serial.println("Client 1 has the highest value");

    } else if (client2IrValue >= client1IrValue && client2IrValue >= client3IrValue) {
        Serial.println("Client 2 has the highest value");

        // Blocking motion stream until button is released
        while (true) {
            leds.blinkAllSimultaneously(200);

            uint8_t currentMotion = motionHandler.processMotion();
            if (currentMotion > 0) {
                client.publish(mqtt_pub_topic, String(currentMotion).c_str());
            }
            delay(20);

            // Escape motion stream on button release
            if (digitalRead(BUTTON_PIN) == HIGH) {
                leds.turnOff();
                client.publish(mqtt_pub_topic, "105");  // 105 signals "stop"
                break;
            }
        }

    } else if (client3IrValue >= client1IrValue && client3IrValue >= client2IrValue) {
        Serial.println("Client 3 has the highest value");
        // Do something for client 3
    }
}


void connectToWiFi() {
    Serial.println("Attempting Wi-Fi connection...");
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    unsigned long connectionTimeout = 10000; // 10 seconds

    while (WiFi.status() != WL_CONNECTED && 
           (millis() - startAttemptTime < connectionTimeout)) {
        Serial.print(".");
        // Optional: blink an LED for feedback
        leds.blinkRed(200);

        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi connected!");
    } else {
        Serial.println("\nWi-Fi connection timed out.");
    }
}


void connectToMQTT() {
    Serial.println("Connecting to MQTT...");
    while (!client.connected()) {
        if (client.connect(mqtt_client_id)) {
            Serial.println("Connected to MQTT broker");
            // Subscribe to the desired topic(s)
            client.subscribe(mqtt_sub_topic);
            Serial.print("Subscribed to topic: ");
            Serial.println(mqtt_sub_topic);
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println("; trying again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(9600);


    pinMode(BUTTON_PIN, INPUT_PULLUP);
    motionHandler.init();
    irControl.init();

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    connectToWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        connectToMQTT();
    }
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
    if (!client.connected()) {
        connectToMQTT();
    }

    client.loop();

    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed");
        leds.blinkAllTogether(200); 
        irControl.update();         
    } else {
        leds.turnOff();
    }

    // Small delay to avoid flooding
    delay(20);
}
