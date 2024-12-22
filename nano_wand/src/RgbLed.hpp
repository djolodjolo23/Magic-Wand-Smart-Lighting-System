#pragma once

#include <Arduino.h>

class RgbLed {
    private:
        const int redPin;
        const int greenPin;
        const int yellowPin;

        bool redPinState = false;
        bool greenPinState = false;
        bool yellowPinState = false;

        unsigned long redPreviousMillis = 0;
        unsigned long greenPreviousMillis = 0;
        unsigned long yellowPreviousMillis = 0;

        int array[3] = {redPin, yellowPin, greenPin};

    public:
        RgbLed(int red, int yellow, int green)
            : redPin(red),
              greenPin(green),
              yellowPin(yellow)
        {
            for (int i = 0; i < 3; i++) {
                pinMode(array[i], OUTPUT);
            }
        }

        void turnOnRed() {
            digitalWrite(redPin, HIGH);
        }
        void turnOnGreen() {
            digitalWrite(greenPin, HIGH);
        }

        void turnOnYellow() {
            digitalWrite(yellowPin, HIGH);
        }

        void blinkRed(unsigned long interval) {
            unsigned long currentMillis = millis();
            if (currentMillis - redPreviousMillis >= interval) {
                redPreviousMillis = currentMillis;
                redPinState = !redPinState; 
                digitalWrite(redPin, redPinState ? HIGH : LOW);
            }
        }

        void blinkGreen(unsigned long interval) {
            unsigned long currentMillis = millis();
            if (currentMillis - greenPreviousMillis >= interval) {
                greenPreviousMillis = currentMillis;
                greenPinState = !greenPinState; 
                digitalWrite(greenPin, greenPinState ? HIGH : LOW);
            }
        }

        void blinkYellow(unsigned long interval) {
            unsigned long currentMillis = millis();
            if (currentMillis - yellowPreviousMillis >= interval) {
                yellowPreviousMillis = currentMillis;
                yellowPinState = !yellowPinState; 
                digitalWrite(yellowPin, yellowPinState ? HIGH : LOW);
            }
        }

        void turnOff() {
            digitalWrite(redPin, LOW);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, LOW);
        }
        
};