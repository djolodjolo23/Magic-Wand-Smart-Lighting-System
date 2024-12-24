#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class LedStrip {
private:
    const int ledPin;
    const int ledCount;
    const int brightness;
    Adafruit_NeoPixel strip;

    uint32_t colors[3]; 

    int currentColorIndex = 0;

public:
    int currentColor = 0;

    LedStrip(int pin, int count, int bright)
        : ledPin(pin),
          ledCount(count),
          brightness(bright),
          strip(count, pin, NEO_GRB + NEO_KHZ800)
    {

        colors[0] = strip.Color(255, 0, 0); // Red
        colors[1] = strip.Color(0, 255, 0); // Green
        colors[2] = strip.Color(0, 0, 255); // Blue
    }

    void setBrightness(int bright) {
        strip.setBrightness(bright);
        strip.show();
    }

    void beginAndShow() {
        strip.begin();
        strip.setBrightness(brightness);
        strip.show();
    }

    void setAllPixels(uint32_t color) {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, color);
        }
        strip.show();
    }

    void showColor(uint32_t color) {
        setAllPixels(color);
    }

    void testFunc(uint8_t val) {
        if (val == 103) { // left
            currentColorIndex = (currentColorIndex + 1) % 3;
            showColor(colors[currentColorIndex]);
        }
        if (val == 104) { // right
            currentColorIndex = (currentColorIndex - 1 + 3) % 3; 
            showColor(colors[currentColorIndex]);
        }
        if (val == 101) { // up
            showColor(colors[currentColorIndex]);
        }
        if (val == 102) { 
            showColor(strip.Color(0, 0, 0));
        }
        if (val >= 0 && val <= 100) {
            setBrightness(val * 2.55);
        }
    }
};
