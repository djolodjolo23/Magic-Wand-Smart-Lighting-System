#pragma once

#include <Arduino.h>

class Leds {
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

        unsigned long previousMillis = 0;

    public:
        Leds(int red, int yellow, int green)
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

        void blinkAllSimultaneously(unsigned long interval) {
            unsigned static long currentLed = 0;
            unsigned long currentMillis = millis();

            if (currentMillis - previousMillis >= interval) {
                currentLed = (currentLed + 1) % 3;
                previousMillis = currentMillis;
            }
            if (currentLed == 0) {
                digitalWrite(redPin, HIGH);
                digitalWrite(yellowPin, LOW);
                digitalWrite(greenPin, LOW);
            } else if (currentLed == 1) {
                digitalWrite(redPin, LOW);
                digitalWrite(yellowPin, HIGH);
                digitalWrite(greenPin, LOW);
            } else if (currentLed == 2) {
                digitalWrite(redPin, LOW);
                digitalWrite(yellowPin, LOW);
                digitalWrite(greenPin, HIGH);
            }
        }

        void blinkAllTogether(unsigned long interval) {
            unsigned long currentMillis = millis();
            static bool allPinsState = false;
            if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                allPinsState = !allPinsState; 
                digitalWrite(redPin, allPinsState ? HIGH : LOW);
                digitalWrite(greenPin, allPinsState ? HIGH : LOW);
                digitalWrite(yellowPin, allPinsState ? HIGH : LOW);
            }
        }
        

        void turnOff() {
            digitalWrite(redPin, LOW);
            digitalWrite(greenPin, LOW);
            digitalWrite(yellowPin, LOW);
        }
        
};