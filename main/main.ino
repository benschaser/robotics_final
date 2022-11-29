#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 1);

// Angle of Inclination
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

// PID Constants
double Kp = 3.44;
double Ki = 0.048;
double Kd = 1.92;

double targetAngle = 0;
double difference = 0;
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

  AFMS.begin();
}
void loop() {
  currTime = millis();
//  Serial.println(currTime - prevTime);
  loopTime = (currTime - prevTime);
  prevTime = currTime;
  
  getCurrentAngle(loopTime);
  calcSpeed();
  Serial.println(motorSpeed);
  prevAngle = currentAngle;
}

void getCurrentAngle(unsigned long elapsedTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accZ = a.acceleration.z;
  accX = a.acceleration.x;
   
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;

  gyroY = g.gyro.y;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*elapsedTime*0.001;

  currentAngle = alpha*(prevAngle + gyroAngle) + (1 - alpha)*accAngle;
}
void calcSpeed() {
  difference = currentAngle - targetAngle;
  difSum = difSum + difference;
  difSum = constrain(difSum, -300, 300);
//  Serial.println(loopTime);
  motorSpeed = Kp*difference + Ki*difSum*loopTime - Kd*(currentAngle - prevAngle)/(loopTime*0.001);
}
