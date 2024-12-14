#pragma once

#include <Arduino.h>    
#include <RP2040_PWM.h>

class IRControl {
    private:
        RP2040_PWM* IR_PWM = NULL;
        const int IR_LED_PIN;
        const int BUTTON_PIN;

    public:
        IRControl(int IrLedPin, int ButtonPin)
            : IR_LED_PIN(IrLedPin),
              BUTTON_PIN(ButtonPin){}

        void init() {
            pinMode(IR_LED_PIN, OUTPUT);
            pinMode(BUTTON_PIN, INPUT_PULLUP);
            IR_PWM = new RP2040_PWM(IR_LED_PIN, 38000, 50.0);
        }

        void update() {
            if (digitalRead(BUTTON_PIN) == LOW) {
                IR_PWM->setPWM(IR_LED_PIN, 38000, 50.0);
            } else {
                IR_PWM->setPWM(IR_LED_PIN, 38000, 0.0);
            }
        }
};