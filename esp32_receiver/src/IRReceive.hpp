#pragma once
#include <Arduino.h>

class IRReceive {
private:
    int irReceiverPin;
    int greednLedPin;
public:
    IRReceive(int irReceiverPin, int greednLedPin)
    : irReceiverPin(irReceiverPin), greednLedPin(greednLedPin) 
    {
        pinMode(irReceiverPin, INPUT);
        pinMode(greednLedPin, OUTPUT);
    }

    void listenForIR() 
    {
        unsigned long previousMillis = millis();
        const long interval = 100;
        while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
                int receiverState = digitalRead(irReceiverPin);
                Serial.print("IR Receiver Output: ");
                Serial.println(receiverState);
                if (receiverState == HIGH) {
                    digitalWrite(greednLedPin, LOW);
                } else {
                    digitalWrite(greednLedPin, HIGH);
                }
                previousMillis = currentMillis;
            }
        }
    }
};

