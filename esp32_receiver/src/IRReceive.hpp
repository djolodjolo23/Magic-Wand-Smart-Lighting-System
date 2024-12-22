#pragma once
#include <Arduino.h>

class IRReceive {
private:
    int irReceiverPinOne;
    int irReceiverPinTwo;
    int irReceiverPinThree;

    int greenLedPin;

    unsigned long previousMillis;

    unsigned long irSignalInterval = 1000;

    int firstIrReceiverCounter = 0;
    int secondIrReceiverCounter = 0;
    int thirdIrReceiverCounter = 0;

public:
    const long interval = 5;
    IRReceive(int irReceiverPinOne, int irReceiverPinTwo, int irReceiverPinThree, int greenLedPin)
    : irReceiverPinOne(irReceiverPinOne), irReceiverPinTwo(irReceiverPinTwo), irReceiverPinThree(irReceiverPinThree), greenLedPin(greenLedPin) 
    {
        pinMode(irReceiverPinOne, INPUT);
        pinMode(irReceiverPinTwo, INPUT);
        pinMode(irReceiverPinThree, INPUT);
        pinMode(greenLedPin, OUTPUT);
    }

    void listenForIR() {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            int receiverState = digitalRead(irReceiverPinOne);
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

    uint8_t testFunc() {

        firstIrReceiverCounter = 0;
        secondIrReceiverCounter = 0;
        thirdIrReceiverCounter = 0;

        unsigned long previousMillis = millis();
        while (millis() - previousMillis <= irSignalInterval) {
            int receiverState1 = digitalRead(irReceiverPinOne);
            int receiverState2 = digitalRead(irReceiverPinTwo);
            int receiverState3 = digitalRead(irReceiverPinThree);

            if (receiverState1 == LOW) {
                digitalWrite(greenLedPin, HIGH);
                firstIrReceiverCounter++;
                Serial.println("Adding to first counter");
            } else {
                digitalWrite(greenLedPin, LOW);
            }
            //if (receiverState2 == LOW) {
                //Serial.println("Adding to second counter");
                //secondIrReceiverCounter++;
            //}
            //if (receiverState3 == LOW) {
                //Serial.println("Adding to third counter");
                //thirdIrReceiverCounter++;
            //}
        }
        Serial.print("First IR Receiver Counter: ");
        Serial.println(firstIrReceiverCounter);
        Serial.print("Second IR Receiver Counter: ");
        Serial.println(secondIrReceiverCounter);
        Serial.print("Third IR Receiver Counter: ");
        Serial.println(thirdIrReceiverCounter);
        if (firstIrReceiverCounter > secondIrReceiverCounter && firstIrReceiverCounter > thirdIrReceiverCounter) {
            Serial.println("First IR Receiver is the highest");
            return 2; // test value
        } else if (secondIrReceiverCounter > firstIrReceiverCounter && secondIrReceiverCounter > thirdIrReceiverCounter) {
            Serial.println("Second IR Receiver is the highest");
            return 2;   // test value
        } else if (thirdIrReceiverCounter > firstIrReceiverCounter && thirdIrReceiverCounter > secondIrReceiverCounter) {
            Serial.println("Third IR Receiver is the highest");
            return 2;  // test value
        }
        return 0;
    }

};

