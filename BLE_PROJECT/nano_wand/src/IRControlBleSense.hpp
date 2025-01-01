#include <Arduino.h>
#include "mbed.h"         // Mbed OS main header
#include "PinNames.h"     // Header where PinName is defined (if not included by default)

class IrControlBleSense {
    private:
        const int IR_LED_PIN;
        mbed::PwmOut *pwm;

    public:
        IrControlBleSense(int IrLedPin)
            : IR_LED_PIN(IrLedPin), pwm(NULL) {}

        void init() {

            // Convert Arduino pin to an Mbed PinName
            PinName pinName = digitalPinToPinName(IR_LED_PIN);

            // Create a PwmOut object on that pin
            pwm = new mbed::PwmOut(pinName);

            // Set a ~38kHz period: 1/38kHz ≈ 26.3us
            pwm->period_us(26); 
            pwm->write(0.0f); // Start off
        }

        void update() {
            pwm->write(0.5f);
            delay(50);
            pwm->write(0.0f);
            delay(5);
        }
};
