#pragma once

//#include <Arduino_LSM6DS3.h> // for rp2040 connect
#include <Arduino_LSM9DS1.h> // for ble sense
#include "Streaming.h"
#include "SensorFusion.h"

class MotionHandler {
    private:

        SF fusion;
        float gx, gy, gz, ax, ay, az;
        float pitch, roll, yaw;
        float deltat;

        static constexpr float ROLL_THRESHOLD = 1.0f;
        static constexpr float PITCH_SPIKE_THRESHOLD = 17.0f;
        static constexpr float YAW_SPIKE_THRESHOLD = 17.0f;

        static constexpr int BUFFER_SIZE = 10;
        float pitchBuffer[BUFFER_SIZE];
        float yawBuffer[BUFFER_SIZE];
        int bufferIndex;

        static constexpr int STABLE_READING_COUNT = 35;
        static constexpr int STABLE_READING_THRESHOLD = 2;
        int stableReadings;
        int lastBrightness;

        bool inRollMode;
        static constexpr unsigned long EVENT_COOLDOWN = 600;
        static constexpr unsigned long ROLL_MODE_TIMEOUT = 1000;
        unsigned long lastEventTime;

        void updateBuffers() {
            pitchBuffer[bufferIndex] = pitch;
            yawBuffer[bufferIndex] = yaw;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        }

        float calculateBrightness() {
            int brightness = (int)(((90.0 - roll) / 180.0) * 100.0);
            return constrain(brightness, 0, 100);
        }

        bool isStableBrightness(int brightness) {
            if (abs(brightness - lastBrightness) <= STABLE_READING_THRESHOLD) {
                stableReadings++;
            } else {
                stableReadings = 0;
            }
            return stableReadings >= STABLE_READING_COUNT;
        }

        public:
            MotionHandler() :
                gx(0), gy(0), gz(0), ax(0), ay(0), az(0),
                pitch(0), roll(0), yaw(0), deltat(0),
                bufferIndex(0), stableReadings(0), lastBrightness(50),
                inRollMode(false), lastEventTime(0) {
                for (int i = 0; i < BUFFER_SIZE; i++) {
                    pitchBuffer[i] = 0.0f;
                    yawBuffer[i] = 0.0f;
                }
            }

            void init() {
                if (!IMU.begin()) {
                    Serial.println("IMU initialization unsuccessful");
                    while (1) { ; }
                } else {
                    Serial.println("IMU initialized successfully");
                }
            }

            String processMotion() {
                if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
                    IMU.readAcceleration(ax, ay, az);
                    IMU.readGyroscope(gx, gy, gz);

                    gx *= DEG_TO_RAD;
                    gy *= DEG_TO_RAD;
                    gz *= DEG_TO_RAD;

                    deltat = fusion.deltatUpdate();
                    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);

                    roll = fusion.getRoll();
                    pitch = fusion.getPitch();
                    yaw = fusion.getYaw();

                    updateBuffers();

                    unsigned long currentTime = millis();

                    float rollDiff = abs(roll - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
                    float pitchDiff = abs(pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
                    float yawDiff = abs(yaw - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]);

                    float maxDiff = rollDiff;
                    char dominantAxis = 'R';

                    if (pitchDiff > maxDiff) {
                        maxDiff = pitchDiff;
                        dominantAxis = 'P';
                    }
                    if (yawDiff > maxDiff) {
                        maxDiff = yawDiff;
                        dominantAxis = 'Y';
                    }

                    if (!inRollMode) {
                        if ((currentTime - lastEventTime > EVENT_COOLDOWN)) {
                            if (dominantAxis == 'R' && maxDiff > ROLL_THRESHOLD && (currentTime - lastEventTime > ROLL_MODE_TIMEOUT)) {
                                inRollMode = true;
                                lastEventTime = currentTime;
                            } else if (dominantAxis == 'P' && maxDiff > PITCH_SPIKE_THRESHOLD) {
                                lastEventTime = currentTime;
                                return (pitch > pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]) ? "UP" : "DOWN";
                            } else if (dominantAxis == 'Y' && maxDiff > YAW_SPIKE_THRESHOLD) {
                                lastEventTime = currentTime;
                                return (yaw > yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]) ? "LEFT" : "RIGHT";
                            }
                        }
                    }

                    if (inRollMode) {
                        int brightness = calculateBrightness();

                        if (isStableBrightness(brightness)) {
                            inRollMode = false;
                        }

                        lastBrightness = brightness;
                        return "BRIGHTNESS: " + String(brightness);

                    }
                }

                return "";
            }

};