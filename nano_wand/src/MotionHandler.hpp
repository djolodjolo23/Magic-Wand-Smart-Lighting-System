#pragma once

//#include <Arduino_LSM6DS3.h>
#include <Arduino_LSM9DS1.h>
#include "Streaming.h"
#include "SensorFusion.h"

class MotionHandler {
private:
    SF fusion;
    float gx, gy, gz;  
    float ax, ay, az;  
    float pitch, roll, yaw; 
    float deltat;

    static constexpr int FILTER_WINDOW = 10;  
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

    // Your existing thresholds
    static constexpr float ROLL_THRESHOLD        = 5.0f;
    static constexpr float PITCH_SPIKE_THRESHOLD = 10.0f;
    static constexpr float YAW_SPIKE_THRESHOLD   = 10.0f;

    // --------- New margin value ------------
    static constexpr float ROLL_MARGIN = 50.0f; 
    // If pitch or yaw are within ROLL_MARGIN of roll, we won't pick roll as dominant

    // Buffers for angle references
    static constexpr int BUFFER_SIZE = 10;
    float pitchBuffer[BUFFER_SIZE];
    float yawBuffer[BUFFER_SIZE];
    int   bufferIndex;

    // Stability / brightness
    static constexpr int STABLE_READING_COUNT     = 35;
    static constexpr int STABLE_READING_THRESHOLD = 4;
    int stableReadings;
    int lastBrightness;

    bool inRollMode;
    static constexpr unsigned long EVENT_COOLDOWN     = 600;
    static constexpr unsigned long ROLL_MODE_TIMEOUT  = 1000;
    unsigned long lastEventTime;

    // ---------------------------------------
    // 1) Time-window-based diff accumulation
    // ---------------------------------------
    // We'll store rollDiff, pitchDiff, yawDiff for a short window (e.g., ~100 ms)
    static constexpr int DIFF_WINDOW_SIZE = 15; 
    float rollDiffBuffer[DIFF_WINDOW_SIZE];
    float pitchDiffBuffer[DIFF_WINDOW_SIZE];
    float yawDiffBuffer[DIFF_WINDOW_SIZE];
    int   diffIndex;    
    bool  diffWindowFull;  

    // ---------------------------------------
    // Helper: accumulate diffs each iteration
    // ---------------------------------------
    void accumulateDiffs(float rollDiff, float pitchDiff, float yawDiff) {
        rollDiffBuffer[diffIndex]  = rollDiff;
        pitchDiffBuffer[diffIndex] = pitchDiff;
        yawDiffBuffer[diffIndex]   = yawDiff;

        diffIndex = (diffIndex + 1) % DIFF_WINDOW_SIZE;
        if (diffIndex == 0) {
            // Once we wrap around, we know the window is full
            diffWindowFull = true;
        }
    }

    // ---------------------------------------
    // Helper: evaluate dominant axis over the window
    // ---------------------------------------
    char evaluateDominantAxisOverWindow() {
        // We can compute an average or a peak. Let's do average here.
        float sumRollDiff  = 0, sumPitchDiff = 0, sumYawDiff = 0;
        int count = diffWindowFull ? DIFF_WINDOW_SIZE : diffIndex; // how many samples we actually have

        if (count == 0) {
            // no data yet
            return 'N'; 
        }

        for (int i = 0; i < count; i++) {
            sumRollDiff  += rollDiffBuffer[i];
            sumPitchDiff += pitchDiffBuffer[i];
            sumYawDiff   += yawDiffBuffer[i];
        }

        float avgRollDiff  = sumRollDiff  / count;
        float avgPitchDiff = sumPitchDiff / count;
        float avgYawDiff   = sumYawDiff   / count;

        // Compare average diffs to thresholds
        bool rollPassed  = (avgRollDiff  > ROLL_THRESHOLD);
        bool pitchPassed = (avgPitchDiff > PITCH_SPIKE_THRESHOLD);
        bool yawPassed   = (avgYawDiff   > YAW_SPIKE_THRESHOLD);

        // If none pass, return 'N'
        if (!rollPassed && !pitchPassed && !yawPassed) {
            return 'N';
        }

        // If only one passes, pick that
        int passCount = (rollPassed?1:0) + (pitchPassed?1:0) + (yawPassed?1:0);
        if (passCount == 1) {
            if (rollPassed)  return 'R';
            if (pitchPassed) return 'P';
            if (yawPassed)   return 'Y';
        }

        // If multiple pass, pick the biggest average diff
        float maxDiff = avgRollDiff;
        char  dominantAxis = 'R';

        // *** Add the margin check for roll *** 
        // Means roll must exceed pitch by ROLL_MARGIN, etc.

        // We'll do standard "which is bigger" logic, 
        // but keep in mind if pitch is close to roll by less than ROLL_MARGIN, skip roll

        // Check pitch
        if (avgPitchDiff > maxDiff) {
            // If roll is bigger but not bigger by margin, we might skip rolling
            maxDiff = avgPitchDiff;
            dominantAxis = 'P';
        } 
        // Check yaw
        if (avgYawDiff > maxDiff) {
            maxDiff = avgYawDiff;
            dominantAxis = 'Y';
        }

        // Additional margin logic if roll is the chosen axis
        if (dominantAxis == 'R') {
            // If pitch is within ROLL_MARGIN => discard roll
            if ((avgRollDiff - avgPitchDiff) < ROLL_MARGIN && avgPitchDiff > ROLL_THRESHOLD) {
                // pitch is close, so let's pick pitch
                dominantAxis = 'P';
            }
            // If yaw is within ROLL_MARGIN => discard roll
            if ((avgRollDiff - avgYawDiff) < ROLL_MARGIN && avgYawDiff > ROLL_THRESHOLD) {
                dominantAxis = 'Y';
            }
        }

        return dominantAxis;
    }

    // ---------------------------------------
    // Rest of your existing helpers
    // ---------------------------------------
    void updateBuffers() {
        pitchBuffer[bufferIndex] = pitch;
        yawBuffer[bufferIndex]   = yaw;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    }

    float calculateBrightness() {
        int brightness = (int)(((90.0f + roll) / 180.0f) * 100.0f);
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

        angleIndex = (angleIndex + 1) % ANGLE_WINDOW;
    }

public:
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
        lastEventTime(0),
        diffIndex(0),
        diffWindowFull(false)
    {
        for (int i = 0; i < BUFFER_SIZE; i++) {
            pitchBuffer[i] = 0.0f;
            yawBuffer[i]   = 0.0f;
        }
        for (int i = 0; i < DIFF_WINDOW_SIZE; i++) {
            rollDiffBuffer[i]  = 0.0f;
            pitchDiffBuffer[i] = 0.0f;
            yawDiffBuffer[i]   = 0.0f;
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

    void debugMotion() {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
            readAndFilterIMU();
            gx *= DEG_TO_RAD;
            gy *= DEG_TO_RAD;
            gz *= DEG_TO_RAD;

            deltat = fusion.deltatUpdate();
            //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
            fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);

            roll  = fusion.getRoll();
            pitch = fusion.getPitch();
            yaw   = fusion.getYaw();

            smoothAngles();

            // Print angles
            Serial.print("Roll: ");
            Serial.print(roll, 2);
            Serial.print("   Pitch: ");
            Serial.print(pitch, 2);
            Serial.print("   Yaw: ");
            Serial.print(yaw, 2);
            Serial.println();
        }
    }

    // Main motion function
    uint8_t processMotion() {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            readAndFilterIMU();

            gx *= DEG_TO_RAD;
            gy *= DEG_TO_RAD;
            gz *= DEG_TO_RAD;

            deltat = fusion.deltatUpdate();
            //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
            fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);

            roll  = fusion.getRoll();
            pitch = fusion.getPitch();
            yaw   = fusion.getYaw();

            smoothAngles();
            updateBuffers();

            unsigned long currentTime = millis();

            // Instead of checking diffs for just the last sample, we 
            //  compute them and store them for a short window
            float rollDiff  = fabs(roll  - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float pitchDiff = fabs(pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float yawDiff   = fabs(yaw   - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]);

            // Accumulate into diff arrays
            accumulateDiffs(rollDiff, pitchDiff, yawDiff);

            // Only evaluate the axis once we have enough samples (or after a short time).
            // Let's evaluate every iteration for demo, but you could check every 10 samples.
            char dominantAxis = evaluateDominantAxisOverWindow();

            if (!inRollMode) {
                if ((currentTime - lastEventTime > EVENT_COOLDOWN)) {
                    if (dominantAxis == 'R' && (currentTime - lastEventTime > ROLL_MODE_TIMEOUT)) {
                        inRollMode = true;
                        lastEventTime = currentTime;
                    } else if (dominantAxis == 'P') {
                        lastEventTime = currentTime;
                        return (pitch > pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]) ? 101 : 102;
                    } else if (dominantAxis == 'Y') {
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
                } else {
                    return 0; // No change
                }
            }
        }
        return 0;
    }
};
