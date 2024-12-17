#pragma once

#include <Arduino.h>

class RgbLed {
    private:
        const int redPin;
        const int greenPin;
        const int yellowPin;
    public:
        RgbLed(int red, int green, int yellow)
            : redPin(red),
              greenPin(green),
              yellowPin(yellow) 
        {
            pinMode(redPin, OUTPUT);
            // pinMode(greenPin, OUTPUT);
            // pinMode(yellowPin, OUTPUT);
        }

        void turnOnRed() {
            digitalWrite(redPin, HIGH);
            // digitalWrite(greenPin, LOW);
            // digitalWrite(yellowPin, LOW);
        }

        void turnOnGreen() {
            digitalWrite(redPin, LOW);
            digitalWrite(greenPin, HIGH);
            digitalWrite(yellowPin, LOW);
        }

        void turnOnYellow() {
            digitalWrite(redPin, LOW);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, HIGH);
        }

        void turnOff() {
            digitalWrite(redPin, LOW);
            // digitalWrite(greenPin, LOW);
            // digitalWrite(yellowPin, LOW);
        }
};