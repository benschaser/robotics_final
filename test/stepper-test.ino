#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(200, 1);

void setup() {
//    startPing();
    AFMS.begin();
    leftMotor->setSpeed(300);
//    rightMotor->setSpeed(150);
    
    
}
void loop() {
  leftMotor->step(1, FORWARD, DOUBLE);
//  rightMotor->step(100, FORWARD, DOUBLE);
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
