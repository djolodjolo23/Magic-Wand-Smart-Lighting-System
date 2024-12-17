#pragma once

#include <Arduino.h>

// a class for testing the IRremote library, which might be needed in case we have to use ble sense for the final project
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
            IrSender.begin(IR_LED_PIN);
            disableLEDFeedback();
        }

        void update() {
            uint8_t sCommand = 0x34;
            uint8_t sRepeats = 0;
            if (digitalRead(BUTTON_PIN) == LOW) {
                Serial.println("Button pressed, sending IR code");
                IrSender.sendNEC(0x00, sCommand, sRepeats);
                delay(200);
            }
        }
};
