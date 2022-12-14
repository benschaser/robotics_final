#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <Kalman.h>

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
double loopTime = 0.0;
double currentAngle = 0;
double prevAngle = 0;
double tau = 0.7;
double alpha = tau / (tau + loopTime);

Kalman kalmanY;
double pitch = 0.0;
double kalAngleY = 0.0;
double gyroYrate = 0.0;
//double prevAngle = 0.0;
double prevPrevAngle = 0.0;

// PID Constants
double Kp = 39.5;
double Ki = 2.2;
double Kd = 0.11;
int inKp = 0;

double targetAngle = 0.0;
double minError = -15.0;
double maxError = 15.0;
double error = 0;
double prevError = 0;
double errSum = 0;
double motorSpeed = 0;
double prevMotorSpeed = 0;
double motorSpeedAvg = 0;

double motorCutoff = 1.0;
double angleThreshold = 0.9;
bool active = false;
int buttonState = 0;

int rDir = 1;
int gDir = 1;
int bDir = 1;
int rVal = 0;
int gVal = 120;
int bVal = 254;

void setup() {
  Wire.begin();
  mpu.begin();

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  AFMS.begin();
  leftMotor->setSpeed(1);
  rightMotor->setSpeed(1);
  TWBR = ((F_CPU/400000l) - 16) / 2;

  pinMode(8, INPUT); // btn
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  startPing();
}

void loop() {
  buttonState = digitalRead(8);
  rVal = rVal + rDir;
  gVal = gVal + gDir;
  bVal = bVal + bDir;

  // for each color, change direction if reached 0 or 255
  if (rVal >= 255 || rVal <= 0) {
    rDir = rDir * -1;
  }

  if (gVal >= 255 || gVal <= 0) {
    gDir = gDir * -1;
  }

  if (bVal >= 255 || bVal <= 0) {
    bDir = bDir * -1;
  }
  RGB_color(rVal, gVal, bVal);
  if (buttonState == 1) {
    active = !active;
    delay(200);
  }
  
  if (active) {
    digitalWrite(5, HIGH);
    digitalWrite(4, HIGH);
    _main();
  }
  else {
    leftMotor->release();
    rightMotor->release();
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
  }
}
void _main() {
  currTime = millis();
  loopTime = (currTime - prevTime);
  loopTime = loopTime / 1000;
  prevTime = currTime;  
  
  getCurrentAngle(loopTime);
  calcSpeed(currentAngle);
  motorSpeed = constrain(motorSpeed, -500, 500);

  if(currentAngle > targetAngle + angleThreshold || currentAngle < targetAngle - angleThreshold) {
    drive();
  }
  
  prevAngle = currentAngle;
  prevMotorSpeed = motorSpeed;
  prevPrevAngle = prevAngle;
  prevAngle = currentAngle;
}

void getCurrentAngle(double elapsedTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//  Kalman Filter
  gyroY = g.gyro.y;
  gyroYrate = g.gyro.y / 131.0;
  pitch = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, loopTime);
  currentAngle = kalAngleY + 3.0;
}

void calcSpeed(double angle) {
  error = targetAngle - angle;
  errSum += error * loopTime;
  errSum = constrain(errSum, -300, 300);
  motorSpeed = Kp * error + Ki * errSum + Kd * (error - prevError) / loopTime;
  prevError = error;
}

void drive() {
  if (motorSpeed > motorCutoff  || motorSpeed < -motorCutoff) {
    Astepper.setSpeed(motorSpeed);
  }
  Astepper.runSpeed();
}

void startPing() {
  tone(7, 500, 600);
  delay(600);
  tone(7, 750, 100);
  delay(100);
  tone(7, 1000, 50);
  delay(50);
  tone(7, 1500, 50);
  delay(50);
  tone(7, 2000, 50);
  delay(50);
  tone(7, 1500, 50);
  delay(50);
  tone(7, 1000, 100);
  delay(100);
  tone(7, 10, 50);
  delay(50);
}

void RGB_color(int red, int green, int blue) {
  analogWrite(11, red);
  analogWrite(10, green);
  analogWrite(9, blue);
}
