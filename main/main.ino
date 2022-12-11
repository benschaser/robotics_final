#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_StepperMotor *leftMotor = AFMS.getStepper(400, 2);
//Adafruit_StepperMotor *rightMotor = AFMS.getStepper(400, 1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

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
double dt = 0.005;
double alpha = tau / (tau + dt);

// PID Constants
double Kp = 8.0;
double Ki = 0.08;
double Kd = 0.8;
double speedFactor = 2.0;

double targetAngle = 0;
double difference = 0;
double prevDif = 0;
double difSum = 0;
double motorSpeed = 0;
double motorSpeedConv = 0;
double RtoD = 57.2957795131;
double motorCutoff = 16.0;
bool active = false;
int buttonState = 0;

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
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  TWBR = ((F_CPU/400000l) - 16) / 2;

  pinMode(8, INPUT); // btn
}

void loop() {
  buttonState = digitalRead(8);
//  Serial.println(buttonState);
  if (buttonState == 1) {
    active = !active;
    delay(200);
  }
  
  if (active) {
    _main();
  }
  else {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
  }
}
void _main() {
  currTime = millis();
//  Serial.println(currTime - prevTime);
  loopTime = (currTime - prevTime);
  loopTime = loopTime / 1000;
  prevTime = currTime;
  dt = loopTime;
  
  getCurrentAngle(loopTime);
  Serial.print(currentAngle);
  Serial.print(' ');
  calcSpeed(currentAngle);
  motorSpeed = constrain(motorSpeed, -1000, 1000);
  motorSpeed = map(motorSpeed, -1000, 1000, -255, 255);
  Serial.println(motorSpeed);
  motorSpeedConv = map(abs(motorSpeed), 0, 150, 50, 255);
  drive();
  
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


}
void calcSpeed(double angle) {
  difference = angle - targetAngle;
  difSum += difference;
  difSum = constrain(difSum, -300, 300);
  motorSpeed = Kp * difference + Ki * difSum * loopTime + Kd * (difference - prevDif) / loopTime;
  prevDif = difference;  
}
void drive() {
//  if (motorSpeed < -motorCutoff) {
//    leftMotor->step(1, BACKWARD, DOUBLE);
//    rightMotor->step(1, FORWARD, DOUBLE);
//  }
//  else if (motorSpeed > motorCutoff) {
//    leftMotor->step(1, FORWARD, DOUBLE);
//    rightMotor->step(1, BACKWARD, DOUBLE);
//  }
//  leftMotor->setSpeed(abs(motorSpeed));
  leftMotor->setSpeed(motorSpeedConv);
  rightMotor->setSpeed(motorSpeedConv);

  if (motorSpeed > 0.0) {
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
  }
  else if (motorSpeed < 0.0) {
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
  }
}
