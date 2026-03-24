#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Motor pins
int L_PWM = 26;
int R_PWM = 25;

// Two MPU6050 sensors
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

// Sensor raw values
int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;

// Timing
const unsigned long sampleInterval = 2000; // 5ms = 200Hz
unsigned long lastSampleTime = 0;

void setup() {
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  Serial.begin(921600);          // fast serial for 200 Hz
  Wire.begin(21, 22);            // SDA=21, SCL=22
  Wire.setClock(400000);         // fast I2C

  // Initialize MPU6050 sensors
  mpu1.initialize();
  mpu2.initialize();

  // Set accelerometer full-scale to ±16g
  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
}

void loop() {
  // Ramp up from 0 → 255, log data during ramp
  rampMotorLog(0, 255, 25000);

  // Hold max speed for 10 seconds, log data
 // holdMaxSpeed(255, 10000);

  // Ramp down from 255 → 0, do NOT log data
  rampMotorNoLog(255, 0, 10000);

  while (1); // Stop after one cycle
}

// ----------------------------------------------------
// Ramp motor and log data
// ----------------------------------------------------
void rampMotorLog(int startPWM, int endPWM, unsigned long duration_ms) {
  const int steps = abs(endPWM - startPWM) + 1;
  unsigned long stepTime = duration_ms / steps; // ms per PWM increment
  int dir = (endPWM > startPWM) ? 1 : -1;
  int pwm = startPWM;

  for (int i = 0; i < steps; i++) {
    analogWrite(R_PWM, pwm);
    analogWrite(L_PWM, 0);

    unsigned long stepStart = millis();
    while (millis() - stepStart < stepTime) {
      sampleSensors(); // log data during ramp
    }

    pwm += dir;
  }
}

// ----------------------------------------------------
// Hold max speed for fixed duration and log data
// ----------------------------------------------------
void holdMaxSpeed(int pwmValue, unsigned long duration_ms) {
  analogWrite(R_PWM, pwmValue);
  analogWrite(L_PWM, 0);

  unsigned long startTime = millis();
  while (millis() - startTime < duration_ms) {
    sampleSensors(); // log data while holding speed
  }
}

// ----------------------------------------------------
// Ramp motor down without logging data
// ----------------------------------------------------
void rampMotorNoLog(int startPWM, int endPWM, unsigned long duration_ms) {
  const int steps = abs(endPWM - startPWM) + 1;
  unsigned long stepTime = duration_ms / steps;
  int dir = (endPWM > startPWM) ? 1 : -1;
  int pwm = startPWM;

  for (int i = 0; i < steps; i++) {
    analogWrite(R_PWM, pwm);
    analogWrite(L_PWM, 0);
    delay(stepTime); // no logging
    pwm += dir;
  }
}

// ----------------------------------------------------
// Function: read both sensors and print to plotter at 200 Hz
// ----------------------------------------------------
void sampleSensors() {
  unsigned long now = micros();
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;

    // Get raw sensor readings
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    // Convert raw accelerometer to g (±16g range → 2048 LSB/g)
    float A1X = ay1 / 2048.0;
    float A1Y = az1 / 2048.0;
    float A1Z = ax1 / 2048.0;

    float A2X = az2 / 2048.0;
    float A2Y = ax2 / 2048.0;
    float A2Z = -1*ay2 / 2048.0;

    // Print CSV: time (ms), A1X, A1Y, A1Z, A2X, A2Y, A2Z
    unsigned long t = millis();
    Serial.print(t); Serial.print(",");
    Serial.print(A1X, 3); Serial.print(",");
    Serial.print(A1Y, 3); Serial.print(",");
    Serial.print(A1Z, 3); Serial.print(",");
    Serial.print(A2X, 3); Serial.print(",");
    Serial.print(A2Y, 3); Serial.print(",");
    Serial.println(A2Z, 3);
  }
}
