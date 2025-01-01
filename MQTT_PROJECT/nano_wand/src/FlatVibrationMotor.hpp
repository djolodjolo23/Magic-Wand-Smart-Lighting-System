#pragma once
#include <Arduino.h>


class FlatVibrationMotor
{
private:
    int m_pin;
    int m_num_vibrations = 0;
    bool m_isVibrating = false;

public:
    FlatVibrationMotor(int pin) : m_pin(pin) {
        pinMode(m_pin, OUTPUT);
    }

    void vibrate(long interval) {
        unsigned long currentMillis = millis();
        static unsigned long previousMillis = 0;

        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            if (m_isVibrating) {
                digitalWrite(m_pin, LOW);
                m_isVibrating = false;
                m_num_vibrations++;
            } else {
                digitalWrite(m_pin, HIGH);
                m_isVibrating = true;
            }
        }
    }

    void vibrateBlocking(long interval) {
        digitalWrite(m_pin, HIGH);
        delay(interval);
        digitalWrite(m_pin, LOW);
    }

    int getNumberOfVibrations() {
        return m_num_vibrations;
    }

    void setNumberOfVibrations(int num) {
        m_num_vibrations = num;
    }

    
};