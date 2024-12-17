#pragma once

#include <Arduino.h>

// Try enabling hardware timer or software PWM mode for IRremote
#define SEND_PWM_BY_TIMER
#include <IRremote.hpp>

class NewIRControl {
    private:
        const int IR_LED_PIN;
        const int BUTTON_PIN;

    public:
        NewIRControl(int IrLedPin, int ButtonPin)
            : IR_LED_PIN(IrLedPin),
              BUTTON_PIN(ButtonPin) {}

        void init() {
            pinMode(BUTTON_PIN, INPUT_PULLUP);
            // Let IrSender handle pin mode
            IrSender.begin(IR_LED_PIN);
        }

        void update() {
            if (digitalRead(BUTTON_PIN) == LOW) {
                Serial.println("Button pressed, sending IR code");
                // Try a standard NEC code
                IrSender.sendNEC(0x00FF00FF, 32);
                delay(200);
            }
        }
};
