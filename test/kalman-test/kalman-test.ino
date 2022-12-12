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
AccelStepper Astepper(forwardstep1, backwardstep1);

Kalman kalmanY;
double gyroY = 0;
double accelX = 0;
double kalAngleY = 0;
double gyroYrate = gyroY / 131.0;
double dt = 0.0;
double currTime = 0;
double prevTime = 0;
double pitch = 0;

void setup() {
  Serial.begin(9600);
  AFMS.begin();

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  else {
    Serial.println("MPU6050 Found!");
  }
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  TWBR = ((F_CPU / 400000UL) - 16) / 2;
  delay(100);
}

void loop() {
  currTime = millis();
  dt = (currTime - prevTime);
  dt = dt / 1000;
  prevTime = currTime;
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  gyroYrate = g.gyro.y / 131.0;
  pitch = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;

  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
//  Serial.println(kalAngleY);
  if (kalAngleY != 0) {
    Astepper.setSpeed(-20 * kalAngleY);
    Astepper.runSpeed();
  }
  
  
}
