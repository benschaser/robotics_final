#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(800, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(800, 1);
double speedNum = 1;

void setup() {
    AFMS.begin();
    TWBR = ((F_CPU/400000l) - 16) / 2;
    leftMotor->setSpeed(120);
    rightMotor->setSpeed(120);
}
void loop() {
  leftMotor->step(1, FORWARD, DOUBLE);
  rightMotor->step(1, BACKWARD, DOUBLE);
}



void startPing() {
  tone(3, 500, 600);
  delay(600);
  tone(3, 750, 100);
  delay(100);
  tone(3, 1000, 50);
  delay(50);
  tone(3, 1500, 50);
  delay(50);
  tone(3, 2000, 50);
  delay(50);
  tone(3, 1500, 50);
  delay(50);
  tone(3, 1000, 100);
  delay(100);
}
