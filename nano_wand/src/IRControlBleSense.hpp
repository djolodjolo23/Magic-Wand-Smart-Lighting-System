#include <Arduino.h>
#include "mbed.h"         // Mbed OS main header
#include "PinNames.h"     // Header where PinName is defined (if not included by default)

class IrControlBleSense {
    private:
        const int IR_LED_PIN;
        const int BUTTON_PIN;
        mbed::PwmOut *pwm;

    public:
        IrControlBleSense(int IrLedPin, int ButtonPin)
            : IR_LED_PIN(IrLedPin), BUTTON_PIN(ButtonPin), pwm(NULL) {}

        void init() {
            pinMode(BUTTON_PIN, INPUT_PULLUP);

            // Convert Arduino pin to an Mbed PinName
            PinName pinName = digitalPinToPinName(IR_LED_PIN);

            // Create a PwmOut object on that pin
            pwm = new mbed::PwmOut(pinName);

            // Set a ~38kHz period: 1/38kHz ≈ 26.3us
            pwm->period_us(26); 
            pwm->write(0.0f); // Start off
        }

        void update() {
            if (digitalRead(BUTTON_PIN) == LOW) {
                pwm->write(0.5f);
                delay(150);
                pwm->write(0.0f);
                delay(150);
            } else {
                pwm->write(0.0f);
            }
        }
};
