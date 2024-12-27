#pragma once

//#include <Arduino_LSM6DS3.h> // for rp2040 connect
#include <Arduino_LSM9DS1.h>  // for ble sense
#include "Streaming.h"
#include "SensorFusion.h"

class MotionHandler {
private:
    SF fusion;
    float gx, gy, gz;  // Filtered gyroscope values
    float ax, ay, az;  // Filtered accelerometer values
    float pitch, roll, yaw; // Smoothed angles
    float deltat;

  
    static constexpr int FILTER_WINDOW = 5;  
    float gxSamples[FILTER_WINDOW];
    float gySamples[FILTER_WINDOW];
    float gzSamples[FILTER_WINDOW];
    float axSamples[FILTER_WINDOW];
    float aySamples[FILTER_WINDOW];
    float azSamples[FILTER_WINDOW];
    int   sampleIndex;        
    bool  buffersInitialized;   

    static constexpr int ANGLE_WINDOW = 5;  
    float rollSamples[ANGLE_WINDOW];
    float pitchSamples[ANGLE_WINDOW];
    float yawSamples[ANGLE_WINDOW];
    int   angleIndex;             
    bool  angleBuffersInitialized; 

    static constexpr float ROLL_THRESHOLD       = 3.0f;
    static constexpr float PITCH_SPIKE_THRESHOLD = 13.0f;
    static constexpr float YAW_SPIKE_THRESHOLD   = 13.0f;

    static constexpr int BUFFER_SIZE = 10;
    float pitchBuffer[BUFFER_SIZE];
    float yawBuffer[BUFFER_SIZE];
    int   bufferIndex;

    // Stability logic
    static constexpr int STABLE_READING_COUNT      = 35;
    static constexpr int STABLE_READING_THRESHOLD  = 2;
    int stableReadings;
    int lastBrightness;

    // Roll mode variables
    bool inRollMode;
    static constexpr unsigned long EVENT_COOLDOWN     = 600;
    static constexpr unsigned long ROLL_MODE_TIMEOUT  = 1000;
    unsigned long lastEventTime;

    void updateBuffers() {
        pitchBuffer[bufferIndex] = pitch;
        yawBuffer[bufferIndex]   = yaw;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    }

    float calculateBrightness() {
        // Example calculation mapping roll from -90 to +90 => brightness 0..100
        int brightness = (int)(((90.0 + roll) / 180.0) * 100.0);
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


    void recordRawSample(float& outVal, float* buffer, float newVal) {
        buffer[sampleIndex] = newVal;

        float sum = 0;
        for (int i = 0; i < FILTER_WINDOW; i++) {
            sum += buffer[i];
        }
        outVal = sum / FILTER_WINDOW;
    }

    void readAndFilterIMU() {
        float gxRaw, gyRaw, gzRaw;
        float axRaw, ayRaw, azRaw;

        IMU.readAcceleration(axRaw, ayRaw, azRaw);
        IMU.readGyroscope(gxRaw, gyRaw, gzRaw);

        if (!buffersInitialized) {
            for (int i = 0; i < FILTER_WINDOW; i++) {
                gxSamples[i] = gxRaw;
                gySamples[i] = gyRaw;
                gzSamples[i] = gzRaw;
                axSamples[i] = axRaw;
                aySamples[i] = ayRaw;
                azSamples[i] = azRaw;
            }
            buffersInitialized = true;
        }

        recordRawSample(gx, gxSamples, gxRaw);
        recordRawSample(gy, gySamples, gyRaw);
        recordRawSample(gz, gzSamples, gzRaw);
        recordRawSample(ax, axSamples, axRaw);
        recordRawSample(ay, aySamples, ayRaw);
        recordRawSample(az, azSamples, azRaw);

        sampleIndex = (sampleIndex + 1) % FILTER_WINDOW;
    }

    void recordAngleSample(float& outVal, float* buffer, float newVal) {
        buffer[angleIndex] = newVal;

        float sum = 0;
        for (int i = 0; i < ANGLE_WINDOW; i++) {
            sum += buffer[i];
        }
        outVal = sum / ANGLE_WINDOW;
    }

    void smoothAngles() {
        if (!angleBuffersInitialized) {
            // First run: fill the angle buffers
            for (int i = 0; i < ANGLE_WINDOW; i++) {
                rollSamples[i]  = roll;
                pitchSamples[i] = pitch;
                yawSamples[i]   = yaw;
            }
            angleBuffersInitialized = true;
        }

        recordAngleSample(roll,  rollSamples,  roll);
        recordAngleSample(pitch, pitchSamples, pitch);
        recordAngleSample(yaw,   yawSamples,   yaw);

        // Move index forward
        angleIndex = (angleIndex + 1) % ANGLE_WINDOW;
    }

public:
    // Constructor
    MotionHandler() 
      : gx(0), gy(0), gz(0), 
        ax(0), ay(0), az(0), 
        pitch(0), roll(0), yaw(0),
        deltat(0),
        sampleIndex(0),
        buffersInitialized(false),
        angleIndex(0),
        angleBuffersInitialized(false),
        bufferIndex(0),
        stableReadings(0), 
        lastBrightness(50),
        inRollMode(false),
        lastEventTime(0)
    {
        // Initialize pitch/yaw buffers to zero
        for (int i = 0; i < BUFFER_SIZE; i++) {
            pitchBuffer[i] = 0.0f;
            yawBuffer[i]   = 0.0f;
        }
    }

    // Initialize the IMU
    void init() {
        if (!IMU.begin()) {
            Serial.println("IMU initialization unsuccessful");
            while (1) { /* halt */ }
        } else {
            Serial.println("IMU initialized successfully");
        }
    }

    // Main motion processing function
    uint8_t processMotion() {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            readAndFilterIMU();

            gx *= DEG_TO_RAD;
            gy *= DEG_TO_RAD;
            gz *= DEG_TO_RAD;

            deltat = fusion.deltatUpdate();
            fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);

            roll  = fusion.getRoll();
            pitch = fusion.getPitch();
            yaw   = fusion.getYaw();

            smoothAngles();

            updateBuffers();

            unsigned long currentTime = millis();

            float rollDiff  = fabs(roll  - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float pitchDiff = fabs(pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float yawDiff   = fabs(yaw   - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]);

            float maxDiff = rollDiff;
            char  dominantAxis = 'R';
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
                    if (dominantAxis == 'R' && maxDiff > ROLL_THRESHOLD &&
                        (currentTime - lastEventTime > ROLL_MODE_TIMEOUT)) 
                    {
                        inRollMode = true;
                        lastEventTime = currentTime;
                    } 
                    else if (dominantAxis == 'P' && maxDiff > PITCH_SPIKE_THRESHOLD) {
                        lastEventTime = currentTime;
                        return (pitch > pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]) ? 101 : 102;
                    } 
                    else if (dominantAxis == 'Y' && maxDiff > YAW_SPIKE_THRESHOLD) {
                        lastEventTime = currentTime;
                        return (yaw > yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]) ? 103 : 104;
                    }
                }
            }

            if (inRollMode) {
                int brightness = (int) calculateBrightness();

                if (isStableBrightness(brightness)) {
                    inRollMode = false;
                }

                if (brightness != lastBrightness) {
                    lastBrightness = brightness;
                    return brightness; 
                }
                else {
                    return 0; // No change
                }
            }
        } 

        return 0; // No motion or no new data
    }
};
