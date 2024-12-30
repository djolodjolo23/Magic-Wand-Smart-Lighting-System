#include <WiFiNINA.h>
#include <PubSubClient.h>

// Wi-Fi and MQTT server credentials
const char* ssid = "Pixel_4585";
const char* password = "123456789";
const char* mqtt_server = "192.168.0.24";

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for serial monitor

    // Connect to Wi-Fi
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Connected to Wi-Fi");

    // Connect to MQTT server
    client.setServer(mqtt_server, 1883);
    if (client.connect("NanoClient")) {
        Serial.println("Connected to MQTT broker");
    } else {
        Serial.println("Failed to connect to MQTT broker");
    }
}

void loop() {
    // Reconnect to MQTT broker if disconnected
    if (!client.connected()) {
        if (client.connect("NanoClient")) {
            Serial.println("Reconnected to MQTT broker");
        } else {
            Serial.println("Failed to reconnect to MQTT broker");
            delay(5000);
            return;
        }
    }

    // Publish a message to the topic
    Serial.println("Publishing message");
    client.publish("test/topic", "Hello from Nano RP2040 Connect!");
    client.loop(); // Process MQTT messages

    delay(5000); // Publish every 5 seconds
}