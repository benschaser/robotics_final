// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
// Requires the Adafruit_Motorshield v2 library
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields

#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 1);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
void forwardstep1() {
  myStepper1->onestep(FORWARD, DOUBLE);
  myStepper2->onestep(BACKWARD, DOUBLE);

}
void backwardstep1() {
  myStepper1->onestep(BACKWARD, DOUBLE);
  myStepper2->onestep(FORWARD, DOUBLE);
}

AccelStepper Astepper1(forwardstep1, backwardstep1); // use functions to step
AccelStepper Astepper2(forwardstep1, backwardstep1); // use functions to step

void setup()
{
   Serial.begin(9600);           // set up Serial library at 9600 bps
   Serial.println("Stepper test!");

  // if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
  //   Serial.println("Could not find Motor Shield. Check wiring.");
  //   while (1);
  // }
  Serial.println("Motor Shield found.");
  AFMS.begin();

  Astepper1.setSpeed(-500);
  Astepper2.setSpeed(-500);
  TWBR = ((F_CPU/400000l) - 16) / 2;
}

void loop() {
   Astepper1.runSpeed();
   delay(0);
}
