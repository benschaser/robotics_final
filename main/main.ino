#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 1);

bool forward = false;
bool stopped = false;

void setup() {
  Serial.begin(9600);

  // Try to initialize!
//  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
//  }
//  else {
//    Serial.println("MPU6050 Found!");
//  }
  mpu.begin();
  AFMS.begin();
  Serial.begin(9600);
  leftMotor->setSpeed(1);
  rightMotor->setSpeed(1);

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  if (forward) {
    leftMotor->step(1, BACKWARD, SINGLE);
    rightMotor->step(1, FORWARD, SINGLE);
  }
  else if (stopped) {
    leftMotor->step(0, FORWARD, SINGLE);
    rightMotor->step(0, BACKWARD, SINGLE);
  }
  else {
    leftMotor->step(1, FORWARD, SINGLE);
    rightMotor->step(1, BACKWARD, SINGLE);
  }
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  double factor = 10 * a.acceleration.x;
  if (factor < 10 && factor > -10) { // close to 0
    factor = 1;
    stopped = true;
    leftMotor->setSpeed(1);
    rightMotor->setSpeed(1);
  }
  else if (factor < 0.0) { // negative
    factor = -factor;
    forward = false;
    stopped  = true;
  }
  else {
    leftMotor->setSpeed(factor);
    rightMotor->setSpeed(factor);
  }
  Serial.println(factor);
  
  delay(10);
}
