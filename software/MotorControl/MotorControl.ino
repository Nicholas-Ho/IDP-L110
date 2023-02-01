#include <Adafruit_MotorShield.h>
#include "LineFollower.h"
#include <NewPing.h>

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

//Ultrasonic
int trigPin = 1;
int echoPin = 0;
int maxDist = 10; //In cm

NewPing sonar(trigPin, echoPin, maxDist);

int blockState = 0; // State of using ultrasonic to detect a block
int slowRatio = 3; // Ratio to slow down the motors by if block is detected (eg if slowRatio is 3, slow down by a factor of 3)

//Ultrasonic intervals
int uSonicInterval = 50; // In milliseconds
long prevMillis = 0;

//Blinky pin
int blinkyPin = 11;
bool blinkyState = false; // True if blinking

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

  // Setup blinky
  pinMode(blinkyPin, OUTPUT);

}

int printCounter = 0;

void loop() 
{ 
  String readingPrint = "";

  // Read line sensors
  for(int i=0; i < 4; i++) {
    lineReadings[i] = digitalRead(linePins[i]);
    readingPrint += String(lineReadings[i]) + " ";
  }

  //Main control system function
  if (!lineReadings)
  {
    Serial.println("Error: Line Readings not available");
    while(1);
  }

  controller.control(lineReadings); //left and right motor proportions are set now

  // Check ultrasonic
  long currMillis = millis();
  if(currMillis - prevMillis >= uSonicInterval) {
    int dist = sonar.ping_cm();
    if(dist <= 8) {
      // Something is within 7 cm
      blockState = 1;
    } else if(dist <= 4) {
      // Something is within 4 cm
      // TODO: Replace below code with block grabbing code
      blockState = 2;
    } else {
      blockState = 0;
    }
  }

  switch(blockState) {
    case 0:
      break;
    
    case 1:
      leftMotorProportion /= slowRatio;
      rightMotorProportion /= slowRatio;
      break;

    case 2:
      // TODO: Replace with block-grabbing code
      leftMotorPropotion = 0;
      rightMotorProportion = 0;
      break;

    default:
      Serial.println("Invalid block state.");
      break;
  }

  if(leftMotorProportion == 0 && rightMotorProportion == 0 && blinkyState == true) {
    digitalWrite(blinkyPin, LOW);
    blinkyState = false;
  } else if(!(leftMotorProportion == 0 && rightMotorProportion == 0) && blinkyState == false) {
    digitalWrite(blinkyPin, HIGH);
    blinkyState = true;
  }

  //Protecting against invalid motor proportions that would cause motor speeds to exceed the max
  if (leftMotorProportion > 1.0)  {leftMotorProportion = 1;}
  if (rightMotorProportion > 1.0) {rightMotorProportion = 1;}

  //Keeping track of travel direction before taking the magnitude of the motor proportions
  bool leftSign = leftMotorProportion >= 0 ? 1 : 0; //1 corresponds to a positive speed (forwards) and 0 to a negative speed (backwards)
  bool rightSign = rightMotorProportion >= 0 ? 1: 0;

  //Setting the motor speeds based on proportions
  int leftMotorSpeed = (int) (fabs(leftMotorProportion)*maxPower);
  int rightMotorSpeed = (int) (fabs(rightMotorProportion)*maxPower); 

  if(printCounter == 100) {
    Serial.println(readingPrint);
    // Serial.println("Left Motor Proportion: ");
    // Serial.println(leftMotorProportion);
    // Serial.println("Left Motor Speed:");
    // Serial.println(leftMotorSpeed);
    // Serial.println("Left Motor Sign: ");
    // Serial.println(leftSign);
    // Serial.println("\n");
    printCounter = 0;
  } else {
    printCounter++;    
  }
  
  //Setting the magnitude of the motor speeds
  leftMotor->setSpeed(leftMotorSpeed);
  rightMotor->setSpeed(rightMotorSpeed);
  // leftMotor->setSpeed(0);
  // rightMotor->setSpeed(0);

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
