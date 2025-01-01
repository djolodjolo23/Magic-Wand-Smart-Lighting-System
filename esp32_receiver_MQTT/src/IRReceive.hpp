#pragma once
#include <Arduino.h>

class IRReceive {
private:
    u_int8_t irReceiverPin;

    unsigned long previousMillis;

    unsigned long irSignalInterval = 1000;

    uint16_t iRReceiveCounter = 0;
  

public:
    IRReceive(u_int8_t irReceiverPin)
    : irReceiverPin(irReceiverPin)
    {
        pinMode(irReceiverPin, INPUT);
    }


    uint8_t listenForIr() {
        iRReceiveCounter = 0;
        unsigned long previousMillis = millis();
        while (millis() - previousMillis <= irSignalInterval) {
            int receiverState = digitalRead(irReceiverPin);
            if (receiverState == LOW) {
                iRReceiveCounter++;
            }
        }
        Serial.println(iRReceiveCounter);
        return iRReceiveCounter;
    }

    uint8_t listenForIrUpdated() {
        iRReceiveCounter = 0;
        if (digitalRead(irReceiverPin) == LOW) {
            unsigned long previousMillis = millis();
            while (millis() - previousMillis <= irSignalInterval) {
                if (digitalRead(irReceiverPin) == LOW) {
                    iRReceiveCounter++;
                }
            }
            return iRReceiveCounter;
        } else {
            return 0;
        }
        
    }
};

