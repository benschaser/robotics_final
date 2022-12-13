#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>

int Kp = 0;
int Kd = 0;

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
void setup() {
  Serial.begin(9600);
  mpu.begin();
  AFMS.begin();
}

void loop() {
  Kp = analogRead(A2);
  Kd = analogRead(A3);
  Serial.print(Kp);
  Serial.print(' ');
  Serial.println(Kd);
}
