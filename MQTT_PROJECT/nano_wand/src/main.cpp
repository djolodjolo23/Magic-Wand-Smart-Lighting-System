#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DSOX.h> // for rp2040 connect
#include "MotionHandler.hpp"
#include "Leds.hpp"
#include "IRControl.hpp"
#include "FlatVibrationMotor.hpp"

// -----------------------------------------------------------------------------
// Wi-Fi & MQTT Credentials
// -----------------------------------------------------------------------------
const char* ssid          = "Pixel_4585";
const char* password      = "123456789";
const char* mqtt_server   = "192.168.0.31";
const char* mqtt_client_id = "NanoClient";  // Unique ID for this device

// Topics
const char* mqtt_sub_topic   = "app/ir_read";
const char* mqtt_pub_topic_1 = "app/motions_1";
const char* mqtt_pub_topic_2 = "app/motions_2";
const char* mqtt_pub_topic_3 = "app/motions_3";

// -----------------------------------------------------------------------------
// Pins & Globals
// -----------------------------------------------------------------------------
const int BUTTON_PIN = 3;
const int VIBRATION_MOTOR_PIN = 5;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
MotionHandler motionHandler;
IRControl irControl(25);
Leds leds(14, 16, 18);
FlatVibrationMotor vibrationMotor(VIBRATION_MOTOR_PIN);

// IR values & flags
int  client1IrValue   = 0;
int  client2IrValue   = 0;
int  client3IrValue   = 0;
bool client1Updated   = false;
bool client2Updated   = false;
bool client3Updated   = false;

// Timing for collecting IR values
unsigned long firstUpdateTime   = 0;             // When the first IR update arrived in the current "round"
bool          timerStarted      = false;         // Indicates if we've started timing
const unsigned long UPDATE_TIMEOUT = 1000;       // 1000 ms (1 second)

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------
void resetClientValues() {
    client1IrValue = 0;
    client2IrValue = 0;
    client3IrValue = 0;
}

void processMotionBlock(String topic) {
    int vibrationState = 0;   // 0: no vibration, 1: first vibration, 2: second vibration, 3: done
    unsigned long vibrationStart = millis();  // Track when vibration started

    while (true) {
        leds.blinkAllSimultaneously(200);

        uint8_t currentMotion = motionHandler.processMotion();
        if (currentMotion > 0) {
            client.publish(topic.c_str(), String(currentMotion).c_str());
        }
        delay(20);

        // Handle non-blocking vibration logic
        if (vibrationState < 3) {
            unsigned long currentMillis = millis();
            if (vibrationState == 0) {
                vibrationMotor.vibrate(100); // Trigger the first short vibration
                vibrationState = 1;
                vibrationStart = currentMillis;
            } else if (vibrationState == 1 && currentMillis - vibrationStart >= 150) {
                vibrationMotor.vibrate(100); // Trigger the second short vibration
                vibrationState = 2;
                vibrationStart = currentMillis;
            } else if (vibrationState == 2 && currentMillis - vibrationStart >= 150) {
                vibrationMotor.vibrate(0); // Ensure motor is off after vibrations
                vibrationState = 3; // Mark vibrations as completed
            }
        }

        // Escape motion stream on button release
        if (digitalRead(BUTTON_PIN) == HIGH) {
            leds.turnOff();
            client.publish(topic.c_str(), "105");  // 105 signals "stop"
            break;
        }
    }
}


// -----------------------------------------------------------------------------
// Compare and Block Function
// -----------------------------------------------------------------------------
void compareAndBlock() {
    // Decide who has the highest IR value:
    if (client1IrValue >= client2IrValue && client1IrValue >= client3IrValue) {
        //Serial.println("Client 1 has the highest value"); // debug
        processMotionBlock(mqtt_pub_topic_1);
    } else if (client2IrValue >= client1IrValue && client2IrValue >= client3IrValue) {
        //Serial.println("Client 2 has the highest value"); // debug
        processMotionBlock(mqtt_pub_topic_2);
    } else if (client3IrValue >= client1IrValue && client3IrValue >= client2IrValue) {
        //Serial.println("Client 3 has the highest value"); // debug
        processMotionBlock(mqtt_pub_topic_3);
    }

    // Reset for next round
    client1Updated = false;
    client2Updated = false;
    client3Updated = false;
    timerStarted   = false;
    resetClientValues();
}

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

        // Check if we already had ANY updated before
        bool wasAnyUpdated = (client1Updated || client2Updated || client3Updated);

        // Update the corresponding IR value
        switch (clientID) {
            case 1:
                client1IrValue = value;
                client1Updated = true;
                break;
            case 2:
                client2IrValue = value;
                client2Updated = true;
                break;
            case 3:
                client3IrValue = value;
                client3Updated = true;
                break;
        }

        // If this is the FIRST update in the current round, note the time
        if (!wasAnyUpdated) {
            firstUpdateTime = millis();
            timerStarted    = true;
        }
    }
}

// -----------------------------------------------------------------------------
// Wi-Fi and MQTT Connection
// -----------------------------------------------------------------------------
void connectToWiFi() {
    Serial.println("Attempting Wi-Fi connection...");
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime   = millis();
    unsigned long connectionTimeout  = 10000; // 10 seconds

    while (WiFi.status() != WL_CONNECTED &&
           (millis() - startAttemptTime < connectionTimeout)) {
        Serial.print(".");
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
            // Subscribe to the desired topic
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

// -----------------------------------------------------------------------------
// Setup & Loop
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);    


    pinMode(BUTTON_PIN, INPUT_PULLUP);
    motionHandler.init();
    irControl.init();

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    // Connect to WiFi, then to MQTT
    connectToWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        connectToMQTT();
    }
}

void loop() {
    // Reconnect logic
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
    if (!client.connected()) {
        connectToMQTT();
    }

    // Process any incoming MQTT messages
    client.loop();

    // Check if all three are updated OR if 1 second has passed
    if (timerStarted) {
        bool allUpdated = (client1Updated && client2Updated && client3Updated);
        bool timeExpired = (millis() - firstUpdateTime >= UPDATE_TIMEOUT);

        if (allUpdated || timeExpired) {
            // Compare IR values and handle the "winning" client
            compareAndBlock();
        }
    }

    // Handle button press for IRControl (if you still want local IR transmissions)
    if (digitalRead(BUTTON_PIN) == LOW) {
        leds.blinkAllTogether(200);
        irControl.update();
    } else {
        leds.turnOff();
    }

    // Small delay to avoid flooding
    delay(20);
}
