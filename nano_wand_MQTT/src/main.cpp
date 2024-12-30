#include <WiFiNINA.h>
#include <PubSubClient.h>

const char* ssid = "Pixel_4585";
const char* password = "123456789";
const char* mqtt_server = "192.168.0.24";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

int client1IrValue = 0;
int client2IrValue = 0;
int client3IrValue = 0;


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

    int clientID = 0;
    int value = 0;

    int separatorIndex = message.indexOf(':');
    if (separatorIndex != -1) {
        clientID = message.substring(0, separatorIndex).toInt();
        value = message.substring(separatorIndex + 1).toInt();
    }

    if (clientID == 1) {
        client1IrValue = value;
    } else if (clientID == 2) {
        client2IrValue = value;
    } else if (clientID == 3) {
        client3IrValue = value;
    }
    if (client1IrValue >= client2IrValue && client1IrValue >= client3IrValue) {
        Serial.println("Client 1 has the highest value");
        //client.publish("app/motions_1", "12345");
    } else if (client2IrValue >= client1IrValue && client2IrValue >= client3IrValue) {
        Serial.println("Client 2 has the highest value");
        //client.publish("app/motions_2", "12345");
    } else if (client3IrValue >= client1IrValue && client3IrValue >= client2IrValue) {
        Serial.println("Client 3 has the highest value");
        //client.publish("app/motions_3", "12345");
    }
}

void setup() {
    Serial.begin(9600);
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