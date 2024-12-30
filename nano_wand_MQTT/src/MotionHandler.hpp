#pragma once

#include <Arduino_LSM6DS3.h> // for rp2040 connect
//#include <Arduino_LSM9DS1.h> // for ble sense
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

    static constexpr int ANGLE_WINDOW = 5; // modifiable
    float rollSamples[ANGLE_WINDOW];
    float pitchSamples[ANGLE_WINDOW];
    float yawSamples[ANGLE_WINDOW];
    int   angleIndex;             
    bool  angleBuffersInitialized; 

    static constexpr float ROLL_THRESHOLD        = 5.0f; // modifiable
    static constexpr float PITCH_SPIKE_THRESHOLD = 10.0f; // modifiable
    static constexpr float YAW_SPIKE_THRESHOLD   = 10.0f; // modifiable

    // margin for not picking roll if pitch or yaw is close
    static constexpr float ROLL_MARGIN = 50.0f;  // modifiable

    static constexpr int BUFFER_SIZE = 10; // modifiable
    float pitchBuffer[BUFFER_SIZE];
    float yawBuffer[BUFFER_SIZE];
    float rollBuffer[BUFFER_SIZE];
    int   bufferIndex;

    static constexpr int STABLE_READING_COUNT     = 50; // modifiable
    static constexpr int STABLE_READING_THRESHOLD = 3; // modifiable
    int stableReadings;
    int lastBrightness;

    bool inRollMode;
    static constexpr unsigned long EVENT_COOLDOWN     = 600; // modifiable
    static constexpr unsigned long ROLL_MODE_TIMEOUT  = 500; // modifiable
    unsigned long lastEventTime;

    float rollOffset = 0.0f;
    float pitchOffset = 0.0f;
    float yawOffset = 0.0f;

    // ---------------------------------------
    // 1) Time-window-based diff accumulation
    // ---------------------------------------
    // We'll store rollDiff, pitchDiff, yawDiff for a short window (e.g., ~100 ms)
    static constexpr int DIFF_WINDOW_SIZE = 15; //window size
    float rollDiffBuffer[DIFF_WINDOW_SIZE];
    float pitchDiffBuffer[DIFF_WINDOW_SIZE];
    float yawDiffBuffer[DIFF_WINDOW_SIZE];
    int   diffIndex;    
    bool  diffWindowFull;  

    // ---------------------------------------
    // Helper: accumulate diffs over the window
    // ---------------------------------------
    void accumulateDiffs(float rollDiff, float pitchDiff, float yawDiff) {
        rollDiffBuffer[diffIndex]  = rollDiff;
        pitchDiffBuffer[diffIndex] = pitchDiff;
        yawDiffBuffer[diffIndex]   = yawDiff;

        diffIndex = (diffIndex + 1) % DIFF_WINDOW_SIZE;
        if (diffIndex == 0) {
            // once we wrap around, the window is full
            diffWindowFull = true;
        }
    }

    // ---------------------------------------
    // Helper: evaluate dominant axis over the window
    // Returns 'R', 'P', 'Y', or 'N' (none)
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

    char evaluateDominantAxisOverWindowPeak() {
        // We'll look for the maximum roll/pitch/yaw diff in the ring buffer
        float peakRollDiff  = 0.0f;
        float peakPitchDiff = 0.0f;
        float peakYawDiff   = 0.0f;

        // Determine how many valid samples we have
        int count = diffWindowFull ? DIFF_WINDOW_SIZE : diffIndex;

        // If we have no samples, return 'N' (none)
        if (count == 0) {
            return 'N';
        }

        // Find the peak (max) values for roll, pitch, yaw diffs
        for (int i = 0; i < count; i++) {
            if (rollDiffBuffer[i]  > peakRollDiff)  peakRollDiff  = rollDiffBuffer[i];
            if (pitchDiffBuffer[i] > peakPitchDiff) peakPitchDiff = pitchDiffBuffer[i];
            if (yawDiffBuffer[i]   > peakYawDiff)   peakYawDiff   = yawDiffBuffer[i];
        }

        // Compare peak diffs to thresholds
        bool rollPassed  = (peakRollDiff  > ROLL_THRESHOLD);
        bool pitchPassed = (peakPitchDiff > PITCH_SPIKE_THRESHOLD);
        bool yawPassed   = (peakYawDiff   > YAW_SPIKE_THRESHOLD);

        // If none pass, return 'N'
        if (!rollPassed && !pitchPassed && !yawPassed) {
            return 'N';
        }

        // If exactly one passes, pick that
        int passCount = (rollPassed ? 1 : 0)
                    + (pitchPassed ? 1 : 0)
                    + (yawPassed   ? 1 : 0);
        if (passCount == 1) {
            if (rollPassed)  return 'R';
            if (pitchPassed) return 'P';
            if (yawPassed)   return 'Y';
        }

        // If multiple pass, pick whichever peak is largest
        float maxDiff = peakRollDiff;
        char dominantAxis = 'R';

        if (peakPitchDiff > maxDiff) {
            maxDiff = peakPitchDiff;
            dominantAxis = 'P';
        }
        if (peakYawDiff > maxDiff) {
            maxDiff = peakYawDiff;
            dominantAxis = 'Y';
        }

        // Additional margin logic if roll is chosen
        if (dominantAxis == 'R') {
            // If pitch is within ROLL_MARGIN => discard roll
            if ((peakRollDiff - peakPitchDiff) < ROLL_MARGIN && peakPitchDiff > ROLL_THRESHOLD) {
                dominantAxis = 'P';
            }
            // If yaw is within ROLL_MARGIN => discard roll
            if ((peakRollDiff - peakYawDiff) < ROLL_MARGIN && peakYawDiff > ROLL_THRESHOLD) {
                dominantAxis = 'Y';
            }
        }

        return dominantAxis;
    }


    // ---------------------------------------
    // Helper: update buffers
    // ---------------------------------------
    void updateBuffers() {
        pitchBuffer[bufferIndex] = pitch;
        yawBuffer[bufferIndex]   = yaw;
        rollBuffer[bufferIndex]  = roll;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    }

    // ---------------------------------------
    // Helper: calculate brightness
    // calculate brightness based on roll, 0-100
    // uses the formula brightness = (90 + roll) / 180 * 100
    // since roll is between -90 and 90, this will give us a value between 0 and 100
    // For RP2040 connect, this formula might need to be adjusted as the range might start from 90 to -90
    // therefore the formula would be brightness = (roll + 90) / 180 * 100
    // ---------------------------------------
    float calculateBrightness() {
        int brightness = (int)(((90.0f + roll) / 180.0f) * 100.0f);
        return constrain(brightness, 0, 100);
    }

    // ---------------------------------------
    // Helper: isStableBrightness
    // check if brightness is stable for a certain number of readings
    // ---------------------------------------
    bool isStableBrightness(int brightness) {
        if (abs(brightness - lastBrightness) <= STABLE_READING_THRESHOLD) {
            stableReadings++;
        } else {
            stableReadings = 0;
        }
        return stableReadings >= STABLE_READING_COUNT;
    }

    // ---------------------------------------
    // Helper: recordRawSample
    // record a raw sample and filter it with moving average
    // ---------------------------------------
    void recordRawSample(float& outVal, float* buffer, float newVal) {
        buffer[sampleIndex] = newVal;
        float sum = 0;
        for (int i = 0; i < FILTER_WINDOW; i++) {
            sum += buffer[i];
        }
        outVal = sum / FILTER_WINDOW;
    }

    // ---------------------------------------
    // Helper: readAndFilterIMU
    // read and filter IMU data for motion processing
    // ---------------------------------------
    void readAndFilterIMU() {
        float gxRaw, gyRaw, gzRaw;
        float axRaw, ayRaw, azRaw;

        IMU.readAcceleration(axRaw, ayRaw, azRaw);
        IMU.readGyroscope(gxRaw, gyRaw, gzRaw);


        // fill up buffers
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
        lastEventTime(0),
        diffIndex(0),
        diffWindowFull(false)
    {
        for (int i = 0; i < BUFFER_SIZE; i++) {
            pitchBuffer[i] = 0.0f;
            yawBuffer[i]   = 0.0f;
            rollBuffer[i]  = 0.0f;
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

    void zeroOrientation() {
        //float currentRoll = fusion.getRoll();
        float currentPitch = fusion.getPitch();
        float currentYaw = fusion.getYaw();

        //rollOffset = currentRoll;
        pitchOffset = currentPitch;
        yawOffset = currentYaw;
    }


    // Debug motion function
    // This function prints the roll, pitch, and yaw values to the Serial Monitor
    void debugMotion() {
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
            // I think Madgwicks is more stable
            fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);

            float rawRoll = fusion.getRoll();
            float rawPitch = fusion.getPitch();
            float rawYaw = fusion.getYaw();

            roll = rawRoll - rollOffset;
            pitch = rawPitch - pitchOffset;
            yaw = rawYaw - yawOffset;

            smoothAngles();
            updateBuffers();

            unsigned long currentTime = millis();

            // Instead of checking diffs for just the last sample, we 
            //  compute them and store them for a short window
            float rollDiff  = fabs(roll - rollBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float pitchDiff = fabs(pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
            float yawDiff   = fabs(yaw   - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]);

            // Accumulate into diff arrays
            accumulateDiffs(rollDiff, pitchDiff, yawDiff);

            char dominantAxis;

            bool usePeakMethod = false;

            if (usePeakMethod) {
                // Use peak method
                dominantAxis = evaluateDominantAxisOverWindowPeak();
            } else {
                // Use average method
                dominantAxis = evaluateDominantAxisOverWindow();
            }

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
