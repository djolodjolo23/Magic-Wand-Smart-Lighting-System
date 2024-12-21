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

    public:
        RgbLed(int red, int green, int yellow)
            : redPin(red),
              greenPin(green),
              yellowPin(yellow) 
        {
            pinMode(redPin, OUTPUT);
            pinMode(greenPin, OUTPUT);
            pinMode(yellowPin, OUTPUT);
        }

        void turnOnRed() {
            digitalWrite(redPin, HIGH);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, LOW);
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

        void blinkRed(long interval) {
            digitalWrite(redPin, HIGH);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, LOW);
            delay(interval);
            digitalWrite(redPin, LOW);
            delay(interval);
        }

        void blinkRedNonBlocking(long startTime, long interval) {
            long currentMilis = millis();
            if (currentMilis - startTime >= interval) {
                if (redPinState) {
                    digitalWrite(redPin, LOW);
                    redPinState = false;
                } else {
                    digitalWrite(redPin, HIGH);
                    redPinState = true;
                }
            }
        }

        void turnOff() {
            digitalWrite(redPin, LOW);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, LOW);
        }
        
};