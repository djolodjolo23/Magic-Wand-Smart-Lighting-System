#include <Arduino_LSM6DS3.h>
#include "Streaming.h"
#include "SensorFusion.h"

SF fusion;

float gx, gy, gz, ax, ay, az;
float pitch, roll, yaw;
float deltat;

static float previousRoll = 0.0f;

const float ROLL_THRESHOLD = 1.0f;          
const float PITCH_SPIKE_THRESHOLD = 17.0f;  
const float YAW_SPIKE_THRESHOLD = 17.0f;   

const int BUFFER_SIZE = 10;
float pitchBuffer[BUFFER_SIZE] = {0};
float yawBuffer[BUFFER_SIZE] = {0};
int bufferIndex = 0;

const int STABLE_READING_COUNT = 20;  // Need 20 stable readings
const int STABLE_READING_THRESHOLD = 2;

int stableReadings = 0;
int lastBrightness = 50;

bool inRollMode = false;  
const unsigned long EVENT_COOLDOWN = 500;
unsigned long lastEventTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!IMU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    while (1) { ; }
  } else {
    Serial.println("IMU initialized successfully");
  }

  for (int i = 0; i < BUFFER_SIZE; i++) {
    pitchBuffer[i] = 0.0f;
    yawBuffer[i] = 0.0f;
  }
}

void loop() {
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

    pitchBuffer[bufferIndex] = pitch;
    yawBuffer[bufferIndex] = yaw;

    unsigned long currentTime = millis();

    
    float rollDiff = abs(roll - previousRoll);
    float pitchDiff = abs(pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]);
    float yawDiff = abs(yaw - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]);

    // finding the dominant axis to trigger the event
    float maxDiff = rollDiff;
    char dominantAxis = 'R'; // R for Roll, P for Pitch, Y for Yaw

    if (pitchDiff > maxDiff) {
      maxDiff = pitchDiff;
      dominantAxis = 'P';
    }
    if (yawDiff > maxDiff) {
      maxDiff = yawDiff;
      dominantAxis = 'Y';
    }

    if (!inRollMode) {
      // only trigger if cooldown has passed
      if ((currentTime - lastEventTime > EVENT_COOLDOWN)) {
        // roll mode triggered
        if (dominantAxis == 'R' && maxDiff > ROLL_THRESHOLD) {
          inRollMode = true;
          // Serial.println("Entering roll mode...");
          lastEventTime = currentTime;
        } else if (dominantAxis == 'P' && maxDiff > PITCH_SPIKE_THRESHOLD) {
        // Pitch trigger
          if (pitch > pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE]) {
            Serial.println("UP");
          } else {
            Serial.println("DOWN");
          }
          lastEventTime = currentTime;
        } else if (dominantAxis == 'Y' && maxDiff > YAW_SPIKE_THRESHOLD) {
          // Yaw trigger
          if (yaw > yawBuffer[(bufferIndex + 1) % BUFFER_SIZE]) {
            Serial.println("LEFT");
          } else {
            Serial.println("RIGHT");
          }
          lastEventTime = currentTime;
        }
      }
    }

    // If in roll mode, continuously map roll angle to brightness
    if (inRollMode) {
       // -90 to 90 degrees mapped to 0 to 100 brightness
      // 0 degrees is neutral
      // 90 degrees is min brightness
      // -90 degrees is max brightness
      // current roll, which can be between -90 and 90 degrees, divided 
      int brightness = (int)(((90.0 - roll) / 180.0) * 100.0); 
      // constrain brightness to 0-100
      brightness = constrain(brightness, 0, 100);

      Serial.println(brightness);

      // exit check, if brightness is stable for 20 readings
      if (abs(brightness - lastBrightness) <= STABLE_READING_THRESHOLD) {
        stableReadings++;
      } else {
        stableReadings = 0; 
      }
        // exit in case of stable readings
      if (stableReadings >= STABLE_READING_COUNT) {
        inRollMode = false;
        // Serial.println("Exiting roll mode...");
      }

      lastBrightness = brightness;
    }

    previousRoll = roll;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    delay(20);
  }
}
