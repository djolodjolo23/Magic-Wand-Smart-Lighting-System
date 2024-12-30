#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Tele2_108703";
const char* password = "q2yymgzk";
const char* mqtt_server = "192.168.0.24";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(9600);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("Connected to WiFi");
    client.setServer(mqtt_server, 1883);
    client.connect("ESP32Client");
}

void loop() {
    if (!client.connected()) {
        client.connect("ESP32Client");
    }
    Serial.println("Publishing message");
    client.loop();

    client.publish("app/ir_read", "1");
    delay(5000); 
}