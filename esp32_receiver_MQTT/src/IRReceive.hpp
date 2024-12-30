#pragma once
#include <Arduino.h>

class IRReceive {
private:
    u_int8_t irReceiverPin;

    unsigned long previousMillis;

    unsigned long irSignalInterval = 1000;

    uint16_t iRReceiveCounter = 0;
  

public:
    const long interval = 5;
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

};

