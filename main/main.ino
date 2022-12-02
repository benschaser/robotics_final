#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 1);
// set offsets


// Angle of Inclination
int accX, accZ;
double accAngle;
int gyroY, gyroRate;
double gyroAngle=0;
unsigned long currTime, prevTime=0;
double loopTime;
double currentAngle = 0;
double prevAngle = 0;
double tau = 0.5;
double dt = 0.01;
double alpha = tau / (tau + dt);

// PID Constants
double Kp = 8.0;
double Ki = 0.05;
double Kd = 4.0;

double targetAngle = 0;
double difference = 0;
double prevDif = 0;
double difSum = 0;
double motorSpeed = 0;

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
//  if (motorSpeed < 1.0 && motorSpeed > -1.0) {
//    
//  }
//  else if (motorSpeed < 0.0) {
//    
//  }
//  else if (motorSpeed > 0.0) {
//    
//  }
  
  prevAngle = currentAngle;
}



void getCurrentAngle(double elapsedTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accZ = a.acceleration.z;
  accX = a.acceleration.x;
   
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;

  gyroY = -g.gyro.y;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*elapsedTime;

  currentAngle = alpha*(prevAngle + gyroAngle) + (1 - alpha)*accAngle;
}
void calcSpeed() {
  difference = currentAngle - targetAngle;
  difSum += difference;
  difSum = constrain(difSum, -300, 300);
//  Serial.println(loopTime);
  motorSpeed = Kp*difference + Ki*difSum*loopTime + Kd*(difference - prevDif)/loopTime;
  prevDif = difference;
}
