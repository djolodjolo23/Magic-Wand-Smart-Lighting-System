#include <Arduino_LSM6DS3.h>
#include "Streaming.h"
#include "SensorFusion.h"

SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;


float pitch, roll, yaw;
float deltat;

static float previousRoll = 0.0f;

const float ROLL_THRESHOLD = 5.0f;          // threshold for roll
const float PITCH_SPIKE_THRESHOLD = 17.0f; // spike threshold 
const float YAW_SPIKE_THRESHOLD = 17.0f;   // spike threshold

const int BUFFER_SIZE = 10; 
float pitchBuffer[BUFFER_SIZE] = {0}; // for checking pitch spikes
float yawBuffer[BUFFER_SIZE] = {0}; // for checking yaw spikes
int bufferIndex = 0;

const unsigned long COOLDOWN_TIME = 600;
unsigned long lastEventTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  
  }

  if (!IMU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    while (1) {
      ;  
    }
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
    //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);

    roll = fusion.getRoll();
    pitch = fusion.getPitch();
    yaw = fusion.getYaw();

    pitchBuffer[bufferIndex] = pitch;
    yawBuffer[bufferIndex] = yaw;

    float rollDifference = roll - previousRoll;
    float pitchChange = pitch - pitchBuffer[(bufferIndex + 1) % BUFFER_SIZE];
    float yawChange = yaw - yawBuffer[(bufferIndex + 1) % BUFFER_SIZE];

    unsigned long currentTime = millis();

    if (abs(rollDifference) > ROLL_THRESHOLD && (currentTime - lastEventTime > COOLDOWN_TIME)) {
      if (rollDifference > 0) {
        Serial.println("ROLL LEFT");
      } else {
        Serial.println("ROLL RIGHT");
      }
      lastEventTime = currentTime;
    }

    if (abs(pitchChange) > PITCH_SPIKE_THRESHOLD && (currentTime - lastEventTime > COOLDOWN_TIME)) {
      if (pitchChange > 0) {
        Serial.println("UP");
      } else {
        Serial.println("DOWN");
      }
      lastEventTime = currentTime;
    }

    if (abs(yawChange) > YAW_SPIKE_THRESHOLD && (currentTime - lastEventTime > COOLDOWN_TIME)) {
      if (yawChange > 0) {
        Serial.println("LEFT");
      } else {
        Serial.println("RIGHT");
      }
      lastEventTime = currentTime;
    }

    previousRoll = roll;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Debug
    //Serial << "Pitch:\t" << pitch << "\tRoll:\t" << roll << "\tYaw:\t" << yaw << endl;

    delay(20); 
  }
}