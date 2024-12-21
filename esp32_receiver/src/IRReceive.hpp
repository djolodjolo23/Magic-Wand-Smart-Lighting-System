#pragma once
#include <Arduino.h>

class IRReceive {
private:
    int irReceiverPin;
    int greenLedPin;
    unsigned long previousMillis;
public:
    const long interval = 5;
    IRReceive(int irReceiverPin, int greenLedPin)
    : irReceiverPin(irReceiverPin), greenLedPin(greenLedPin) 
    {
        pinMode(irReceiverPin, INPUT);
        pinMode(greenLedPin, OUTPUT);
    }

    void listenForIR() {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            int receiverState = digitalRead(irReceiverPin);
            Serial.print("IR Receiver Output: ");
            Serial.println(receiverState);
            if (receiverState == HIGH) {
                digitalWrite(greenLedPin, LOW);
            } else {
                digitalWrite(greenLedPin, HIGH);
            }
            previousMillis = currentMillis;
        }
    }
};

