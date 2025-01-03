#pragma once
#include <Arduino.h>

class IRReceive {
private:
    u_int8_t irReceiverPinOne;
    u_int8_t irReceiverPinTwo;
    u_int8_t irReceiverPinThree;

    unsigned long previousMillis;

    unsigned long irSignalInterval = 1000;

    uint16_t firstIrReceiverCounter = 0;
    uint16_t secondIrReceiverCounter = 0;
    uint16_t thirdIrReceiverCounter = 0;

public:
    const long interval = 5;
    IRReceive(u_int8_t irReceiverPinOne, u_int8_t irReceiverPinTwo, u_int8_t irReceiverPinThree)
    : irReceiverPinOne(irReceiverPinOne), irReceiverPinTwo(irReceiverPinTwo), irReceiverPinThree(irReceiverPinThree)
    {
        pinMode(irReceiverPinOne, INPUT);
        pinMode(irReceiverPinTwo, INPUT);
        pinMode(irReceiverPinThree, INPUT);
    }

    uint8_t listenForIr() {

        firstIrReceiverCounter = 0;
        secondIrReceiverCounter = 0;
        thirdIrReceiverCounter = 0;

        unsigned long previousMillis = millis();
        while (millis() - previousMillis <= irSignalInterval) {
            int receiverState1 = digitalRead(irReceiverPinOne);
            int receiverState2 = digitalRead(irReceiverPinTwo);
            int receiverState3 = digitalRead(irReceiverPinThree);

            if (receiverState1 == LOW) {
                firstIrReceiverCounter++;
                Serial.println("Adding to first counter");
            } 
            if (receiverState2 == LOW) {
                Serial.println("Adding to second counter");
                secondIrReceiverCounter++;
            } 
            if (receiverState3 == LOW) {
                Serial.println("Adding to third counter");
                thirdIrReceiverCounter++;
            } 
        }
        Serial.print("First IR Receiver Counter: ");
        Serial.println(firstIrReceiverCounter);
        Serial.print("Second IR Receiver Counter: ");
        Serial.println(secondIrReceiverCounter);
        Serial.print("Third IR Receiver Counter: ");
        Serial.println(thirdIrReceiverCounter);
        if (firstIrReceiverCounter > secondIrReceiverCounter && firstIrReceiverCounter > thirdIrReceiverCounter) {
            Serial.println("First IR Receiver is the highest");
            return 106; // code for successful connection
        } else if (secondIrReceiverCounter > firstIrReceiverCounter && secondIrReceiverCounter > thirdIrReceiverCounter) {
            Serial.println("Second IR Receiver is the highest");
            return 107;   // code for successful connection
        } else if (thirdIrReceiverCounter > firstIrReceiverCounter && thirdIrReceiverCounter > secondIrReceiverCounter) {
            Serial.println("Third IR Receiver is the highest");
            return 108;  // code for successful connection
        }
        return 109; // code for unsuccessful connection
    }

};

