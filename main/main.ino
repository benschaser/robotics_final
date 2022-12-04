#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(600, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(600, 1);

// Angle of Inclination
double accX, accZ;
double accAngle;
int gyroY, gyroRate;
double gyroAngle = 0;
double gyroAngleNext = 0;
double gyroAnglePrev = 0;
unsigned long currTime, prevTime = 0;
double loopTime;
double currentAngle = 0;
double prevAngle = 0;
double prevPrevAngle = 0;
double nextAngle = 0;
double tau = 0.7;
double dt = 0.004;
double alpha = tau / (tau + dt);

// PID Constants
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double targetAngle = 0;
double difference = 0;
double prevDif = 0;
double difSum = 0;
double motorSpeed = 0;
double RtoD = 57.2957795131;

void setup() {
  Serial.begin(9600);
  mpu.begin();

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //  mpu.setXAccelOffset(315);
  //  mpu.setYAccelOffset(993);
  //  mpu.setZAccelOffset(415);
  //  mpu.setXGyroOffset(93);
  //  mpu.setYGyroOffset(-23);
  //  mpu.setZGyroOffset(-11);

  AFMS.begin();
}

void loop() {
  currTime = millis();
  //  Serial.println(currTime - prevTime);
  loopTime = (currTime - prevTime);
  loopTime = loopTime / 1000;
  prevTime = currTime;
  
  getCurrentAngle(loopTime);
  Serial.println(currentAngle);
  calcSpeed();
//  Serial.println(motorSpeed);
  //
//  if ((currentAngle + prevAngle + prevPrevAngle) / 3 < -1.0) {
//    leftMotor->setSpeed(motorSpeed);
//    rightMotor->setSpeed(motorSpeed);
//    leftMotor->step(1, BACKWARD, DOUBLE);
//    rightMotor->step(1, FORWARD, DOUBLE);
//  }
//  else if ((currentAngle + prevAngle + prevPrevAngle) / 3 > 1.0) {
//    leftMotor->setSpeed(motorSpeed);
//    rightMotor->setSpeed(motorSpeed);
//    leftMotor->step(1, FORWARD, DOUBLE);
//    rightMotor->step(1, BACKWARD, DOUBLE);
//  }
  prevPrevAngle = prevAngle;
  prevAngle = currentAngle;
}



void getCurrentAngle(double elapsedTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accZ = a.acceleration.z;
  accX = a.acceleration.x;
//  Serial.print(accX);
//  Serial.print("  ");
//  Serial.print(accZ);
//  Serial.print('\n');

  accAngle = atan2(accX, accZ) * RtoD;

  gyroY = g.gyro.y;

  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate * elapsedTime;
  currentAngle = tau * (prevAngle + gyroAngle*dt) + (1 - tau) * accAngle;
//  Serial.println(accAngle);

  
//  alternative formula - leapfrog method :)
//  gyroAngleNext = (gyroAnglePrev + gyroAngle) / 2 + accAngle*elapsedTime;
//  nextAngle = currentAngle;

}
void calcSpeed() {
  difference = currentAngle - targetAngle;
  difSum += difference;
  difSum = constrain(difSum, -300, 300);
  motorSpeed = Kp * difference + Ki * difSum * loopTime + Kd * (difference - prevDif) / loopTime;
  prevDif = difference;
}
