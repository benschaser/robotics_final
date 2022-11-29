#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"

Adafruit_MPU6050 mpu;

int accX, accZ;
double accAngle;
int gyroY, gyroRate;
double gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;
double currentAngle = 0;
double prevAngle = 0;
double tau = 0.1;
double dt = 0.01;
double alpha = tau / (tau + dt);


void setup() {
  Serial.begin(9600);
  mpu.begin();

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  currTime = millis();
  loopTime = (currTime - prevTime) / 1000;
  prevTime = currTime;
  
  accZ = a.acceleration.z;
  accX = a.acceleration.x;
   
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
//  Serial.println("Acc ", accAngle);

  gyroY = g.gyro.y;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime;

  // complementary filter (high and low pass filter)
  currentAngle = alpha*(prevAngle + gyroAngle) + (1 - alpha)*accAngle;
  
  prevAngle = currentAngle;
  Serial.println(currentAngle);
}
