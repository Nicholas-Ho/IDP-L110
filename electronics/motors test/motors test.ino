// Motor Sketch, just get it to turn, and change direction


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


int main() {
  // put your main code here, to run repeatedly:
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
  AFMS.begin();
  myMotor->setSpeed(200);
  myMotor->run(FORWARD);
}
