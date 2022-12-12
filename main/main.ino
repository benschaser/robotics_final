#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 1);
void forwardstep1() {
  leftMotor->onestep(BACKWARD, DOUBLE);
  rightMotor->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {
  leftMotor->onestep(FORWARD, DOUBLE);
  rightMotor->onestep(BACKWARD, DOUBLE);
}
AccelStepper Astepper(forwardstep1, backwardstep1); // use functions to step

// Angle of Inclination
double accX, accZ;
double accAngle;
int gyroY, gyroRate;
double gyroAngle = 0;
double gyroAngleNext = 0;
double gyroAnglePrev = 0;
unsigned long currTime, prevTime = 0;
double loopTime = 0.004;
double currentAngle = 0;
double prevAngle = 0;
double tau = 0.7;
//double dt = 0.005;
double alpha = tau / (tau + loopTime);

// PID Constants
double Kp = 7.5;
double Ki = 0.00;
double Kd = 1.0;

double targetAngle = 4.5;
double minError = -11.5;
double maxError = 20.5;
double error = 0;
double prevError = 0;
double errSum = 0;
double motorSpeed = 0;
double prevMotorSpeed = 0;
double prevPrevMotorSpeed = 0;
double motorSpeedAvg = 0;
double motorSpeedConv = 0;
double RtoD = 57.2957795131;
double motorCutoff = 25.0;
bool active = false;
int buttonState = 0;

void setup() {
//  Serial.begin(9600);
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
  leftMotor->setSpeed(1);
  rightMotor->setSpeed(1);
  TWBR = ((F_CPU/400000l) - 16) / 2;

  pinMode(8, INPUT); // btn
}

void loop() {
//  buttonState = digitalRead(8);
////  Serial.println(buttonState);
//  if (buttonState == 1) {
//    active = !active;
//    delay(200);
//  }
//  
//  if (active) {
//    _main();
//  }
  _main();
}
void _main() {
  currTime = millis();
//  Serial.println(currTime - prevTime);
  loopTime = (currTime - prevTime);
//  Serial.println(loopTime);
  loopTime = loopTime / 1000;
  prevTime = currTime;
//  dt = loopTime;
  
  
  getCurrentAngle(loopTime);
//  Serial.print(currentAngle);
//  Serial.print(' ');
//  Serial.print(targetAngle);
//  Serial.print(' ');
  calcSpeed(currentAngle);
  motorSpeed = constrain(motorSpeed, -500, 500);
//  motorSpeed = map(motorSpeed, -1000, 1000, -500, 500);
//  Serial.println(motorSpeedAvg);
//  motorSpeedConv = map(abs(motorSpeed), 0, 150, 50, 255);
  drive();
  
  prevAngle = currentAngle;
//  prevPrevMotorSpeed = prevMotorSpeed;
  prevMotorSpeed = motorSpeed;
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
  currentAngle = tau * (prevAngle + gyroAngle*loopTime) + (1 - tau) * accAngle;
}
void calcSpeed(double angle) {
  error = 4 * constrain(targetAngle - angle, minError, maxError);
  errSum += error * loopTime;
  errSum = constrain(errSum, -300, 300);
  motorSpeed = Kp * error + Ki * errSum + Kd * (error - prevError) / loopTime;
  prevError = error;  
//  motorSpeedAvg = 0.2 * prevMotorSpeed + 0.8 * motorSpeed;
//  Serial.println(motorSpeed);
}
void drive() {
  if (motorSpeed > motorCutoff  || motorSpeed < -motorCutoff) {
    Astepper.setSpeed(-motorSpeed);
    Astepper.runSpeed();
  }
  else {
    leftMotor->release();
    rightMotor->release();
  }
  
}
