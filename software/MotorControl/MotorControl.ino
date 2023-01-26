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
LineFollower controller = LineFollower(leftMotorProportion, rightMotorProportion);

uint8_t maxPower = 220; // 255 is too much

//Assign line sensor pins (left to right)
int linePins[4] = {4, 5, 6, 7};
int lineReadings[4] = {0, 0, 0, 0};

void setup() 
{

  Serial.begin(9600);
  if (!motor_shield.begin())  
  {

    Serial.println("Could not find motors.");
    
  }  
  else 
  {
    Serial.println("Motors online.");
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  controller.turnLeft(0);

  // Setup line sensors
  pinMode(linePins[0], INPUT);
  pinMode(linePins[1], INPUT);
  pinMode(linePins[2], INPUT);
  pinMode(linePins[3], INPUT);

}

void loop() 
{ 
  String readingPrint = "";
  // Read line sensors
  for(int i=0; i < 3; i++) {
    lineReadings[i] = digitalRead(linePins[i]);
    readingPrint += String(lineReadings[i]) + " ";
  }
  lineReadings[3] = 0;
  readingPrint += String(lineReadings[3])

  Serial.println(readingPrint);

  //Main control system function
  if (!lineReadings)
  {
    Serial.println("Error: Line Readings not available");
    while(1);
  }
  controller.control(lineReadings); //left and right motor proportions are set now
  
  //Protecting against invalid motor proportions that would cause motor speeds to exceed the max
  if (leftMotorProportion > 1.0)  {leftMotorProportion = 1;}
  if (rightMotorProportion > 1.0) {rightMotorProportion = 1;}

  //Keeping track of travel direction before taking the magnitude of the motor proportions
  bool leftSign = leftMotorProportion >= 0 ? 1 : 0; //1 corresponds to a positive speed (forwards) and 0 to a negative speed (backwards)
  bool rightSign = rightMotorProportion >= 0 ? 1: 0;

  // //Setting the motor speeds based on proportions
  int leftMotorSpeed = (int) (fabs(leftMotorProportion)*maxPower);
  int rightMotorSpeed = (int) (fabs(rightMotorProportion)*maxPower); 

  Serial.println("Left Motor Proportion: ");
  Serial.println(leftMotorProportion);
  Serial.println("Left Motor Speed:");
  Serial.println(leftMotorSpeed);
  Serial.println("Left Motor Sign: ");
  Serial.println(leftSign);
  Serial.println("\n");
  
  //Setting the magnitude of the motor speeds
  // leftMotor->setSpeed(leftMotorSpeed);
  // rightMotor->setSpeed(rightMotorSpeed);
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

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
