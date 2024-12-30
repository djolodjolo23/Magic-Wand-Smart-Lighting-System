#include <WiFiNINA.h>
#include <PubSubClient.h>

const char* ssid = "Pixel_4585";
const char* password = "123456789";
const char* mqtt_server = "192.168.0.24";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

String messages[3]= {};

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message: ");
    Serial.println(message);
    if (String(topic) == "app/ir_read") {
        Serial.println("IR read message received");
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Connected to Wi-Fi");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("NanoClient")) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }

    // Subscribe to the topic
    client.subscribe("app/ir_read");
    Serial.println("Subscribed to topic: app/ir_read");
}

void loop() {
    
    client.loop();


    // Publish a message to the topic
    // Serial.println("Publishing message");
    // client.publish("test/topic", "Hello from Nano RP2040 Connect!");
    // client.loop(); // Process MQTT messages

    // delay(5000); // Publish every 5 seconds
}