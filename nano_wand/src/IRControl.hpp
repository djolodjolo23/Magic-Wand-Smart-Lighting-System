#pragma once

#include <Arduino.h>

class IRControl {
    private:
        const int IR_LED_PIN;
        const int BUTTON_PIN;

        // The following parameters are chosen to achieve ~38kHz:
        // frequency = system_clock / (clkdiv * (top + 1))
        // system_clock (default) = 125 MHz
        // We want 38 kHz, so:
        // 38,000 = 125,000,000 / (clkdiv * (top+1))
        // Solve for (clkdiv * (top+1)) = 125,000,000 / 38,000 ≈ 3289.47
        //
        // Let's pick a convenient top (wrap value):
        // If we choose top = 999 (for a nice 1000 counts):
        // (top+1) = 1000
        
        // clkdiv = 3289.47 / 1000 ≈ 3.289
        //
        // We'll use:
        // top = 999 (so counts go from 0 to 999)
        // clkdiv = 3.29 (approx)
        //
        // Duty cycle of 50% = level = top/2 = 500
        uint16_t pwmTop = 999;      // wrap value
        float pwmDiv = 3.29;        // clock divider for ~38kHz
        uint16_t dutyOn = 500;      // duty for ~50%
        uint16_t dutyOff = 0;       // duty for off

        uint slice_num;
        uint channel;

    public:
        IRControl(int IrLedPin, int ButtonPin)
            : IR_LED_PIN(IrLedPin),
              BUTTON_PIN(ButtonPin) {}

        void init() {
            pinMode(BUTTON_PIN, INPUT_PULLUP);

            // Set IR_LED_PIN to PWM
            gpio_set_function((uint)IR_LED_PIN, GPIO_FUNC_PWM);

            // Determine PWM slice and channel
            slice_num = pwm_gpio_to_slice_num(IR_LED_PIN);
            channel = pwm_gpio_to_channel(IR_LED_PIN);

            // Configure the PWM frequency
            pwm_set_wrap(slice_num, pwmTop);
            pwm_set_clkdiv(slice_num, pwmDiv);

            pwm_set_chan_level(slice_num, channel, dutyOff);

            pwm_set_enabled(slice_num, true);
        }

        void update() {
            if (digitalRead(BUTTON_PIN) == LOW) {
                pwm_set_chan_level(slice_num, channel, dutyOn);
                delay(150);
                pwm_set_chan_level(slice_num, channel, dutyOff);
                delay(150);
            } else {
                pwm_set_chan_level(slice_num, channel, dutyOff);
            }
        }
};
