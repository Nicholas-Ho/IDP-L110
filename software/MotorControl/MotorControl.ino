#include <Adafruit_MotorShield.h>
#include "LineFollower.h"

//Instantiate MotorShield object
Adafruit_MotorShield motor_shield = Adafruit_MotorShield(0x60);

//Assign left and right motor pin channels
int leftPin = 1;
int rightPin = 2;

//Instantiate the left and right DC motors
Adafruit_DCMotor* leftMotor = motor_shield.getMotor(leftPin);
Adafruit_DCMotor* rightMotor = motor_shield.getMotor(rightPin);

//Initialising the variables representing the output of the controller (proportions of maximum power)
float leftMotorProportion, rightMotorProportion;

//Instantiate a controller object, passing in references to the left and right motor objects
LineFollower controller = LineFollower(&leftMotorProportion, &rightMotorProportion);

uint8_t maxPower = 255;
int lineReadings[4];

void setup() 
{

  Serial.begin(9600);
  if (!motor_shield.begin())  
  {

    Serial.println("Could not find motors.")
    
  }  
  else 
  {
    Serial.println("Motors online.")
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

}

void loop() 
{ 
  //Main control system function
  if (!lineReadings)
  {
    Serial.println("Error: Line Readings not available");
    while(1);
  }
  controller.control(lineReadings); //left and right motor proportions are set now

  //Protecting against overflow of the 8-bit motor speeds
  if (leftMotorProportion > 1.0 || rightMotorProportion > 1.0)
  {
    Serial.println("Invalid control system input");
    while(1);
  }

  //Keeping track of travel direction before taking the magnitude of the motor proportions
  bool leftSign = leftMotorProportion >= 0 ? 1 : 0; //1 corresponds to a positive speed (forwards) and 0 to a negative speed (backwards)
  bool rightSign = rightMotorProportion >= 0 ? 1: 0;

  //Setting the motor speeds based on proportions
  uint8_t leftMotorSpeed = (uint8_t) abs(leftMotorProportion)*maxPower;
  uint8_t rightMotorSpeed = (uint8_t) abs(rightMotorProportion)*maxPower; 

  //Setting the magnitude of the motor speeds
  leftMotor->setSpeed(leftMotorSpeed);
  rightMotor->setSpeed(rightMotorSpeed);

  //Using the sign of the motor proportion to set the direction of motion for each wheel
  switch(leftSign)
  {
    case 0:
      leftMotor->run(BACKWARD);
      break;

    case 1: 
      leftMotor->run(FORWARD);
      break;

    default:
      leftMotor->run(RELEASE);
      break;

  }

  switch(rightSign)
  {
    case 0:
      rightMotor->run(BACKWARD);
      break;

    case 1: 
      rightMotor->run(FORWARD);
      break;

    default:
      rightMotor->run(RELEASE);
      break;

  }

}
