#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

void setup() {
    Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
    Adafruit_StepperMotor *leftMotor = AFMS.getStepper(200, 2);

    startPing();
    leftMotor->setSpeed(120);
    delay(3000);
    leftMotor->setSpeed(0);
    
}
void loop() {

}



void startPing() {
  tone(2, 500, 600);
  delay(600);
  tone(2, 750, 100);
  delay(100);
  tone(2, 1000, 50);
  delay(50);
  tone(2, 1500, 50);
  delay(50);
  tone(2, 2000, 50);
  delay(50);
  tone(2, 1500, 50);
  delay(50);
  tone(2, 1000, 100);
  delay(100);
}
