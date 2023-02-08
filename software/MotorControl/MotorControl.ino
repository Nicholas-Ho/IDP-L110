#include <Adafruit_MotorShield.h>
#include "LineFollower.h"
#include "ColourDetector.h"
#include <NewPing.h>

#define triggerPinTunnel 3
#define echoPinTunnel 2
#define MAX_DISTANCE_TUNNEL 20 //in centimetres

//Instantiate MotorShield object
Adafruit_MotorShield motor_shield = Adafruit_MotorShield(0x60);

//Assign left and right motor pin channels
const int leftPin = 1;
const int rightPin = 2;

//Instantiate the left and right DC motors
Adafruit_DCMotor* leftMotor = motor_shield.getMotor(leftPin);
Adafruit_DCMotor* rightMotor = motor_shield.getMotor(rightPin);

//Initialising the variables representing the output of the controller (proportions of maximum power)
float leftMotorProportion = 0.5;
float rightMotorProportion = 0.5;

//Instantiate a controller object, passing in references to the left and right motor objects
LineFollower controller = LineFollower(leftMotorProportion, rightMotorProportion);

const uint8_t maxPower = 220; // 255 is too much

//Assign line sensor pins (left to right)
const int linePins[4] = {4, 5, 6, 7};
int lineReadings[4] = {0, 0, 0, 0};

//Assign Tunnel Ultrasonic pins
const int triggerPinTun = 3;
const int echoPinTun = 2;
const int MAX_DISTANCE_T = 20; //in centimetres

//Tunnel Ultrasonic
bool inTunnel = true;
NewPing sonarTunnel(triggerPinTunnel, echoPinTunnel, MAX_DISTANCE_TUNNEL);
void setMotorProportions(float&, float&);

//Block Ultrasonic
const int trigPinB = 1;
const int echoPinB = 0;
const int maxDistB = 10; //In cm

NewPing sonarBlock(trigPinB, echoPinB, maxDistB);

int blockState = 0; // State of using ultrasonic to detect a block
const int slowRatio = 3; // Ratio to slow down the motors by if block is detected (eg if slowRatio is 3, slow down by a factor of 3)

//Block Ultrasonic intervals
const int uSonicInterval = 50; // In milliseconds
long prevMillis = 0;

// Pins for colour detection
const int colourPinIn = A1; // Analog In
const int bluePinOut = 13; // Digital Out
const int redPinOut = 12; // Digital Out

int colourSensorVal = 0;
Colour colour = 0;

//Instantiate a colour detector object
ColourDetector detector = ColourDetector();

//Blinky pin
const int blinkyPin = 11;
bool blinkyState = false; // True if blinking

//Push Button
uint8_t buttonPressed = 1;
int buttonPin = 10;

int startup = 0; //Start up sequence state (0 -> Button not pressed, 1 -> Button pressed, robot needs to move, 2 -> End of start up sequence)

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

  // Setup line sensors
  pinMode(linePins[0], INPUT);
  pinMode(linePins[1], INPUT);
  pinMode(linePins[2], INPUT);
  pinMode(linePins[3], INPUT);

  //Setup colour LEDs
  pinMode(bluePinOut, OUTPUT);
  pinMode(redPinOut, OUTPUT);

  // Setup blinky
  pinMode(blinkyPin, OUTPUT);

  //Setup push button
  pinMode(buttonPin, INPUT_PULLUP);

}

int printCounter = 0;

void loop() 
{ 
  // //Read button input every 50ms
  // while (startup == 0)
  // { 
  //   buttonPressed = digitalRead(buttonPin); //Break out of loop if we read LOW on buttonPin 
    // if(buttonPressed == LOW)
    // {
    //   startup = 1;
    //   break;
    // }
  //   delay(50); 
  // }

  // while(startup == 1)
  // {
  //   //Run forward for 2 seconds
  //   leftMotor-> setSpeed(120);
  //   rightMotor -> setSpeed(120);
  //   leftMotor -> run(FORWARD);
  //   rightMotor -> run(FORWARD);
  //   delay(2000);

  //   //Turn to the left (2 seconds)    
  //   leftMotor -> run(BACKWARD)
  //   delay(2000);

  //   //Stop  
  //   leftMotor -> run(RELEASE);
  //   rightMotor -> run(RELEASE);
    // startup = 2;
  // }

  String readingPrint = "";

  // LINE CONTROL
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

  if(!inTunnel) //set motor proportions based on line sensor input
  {
    //controller.control(lineReadings); //left and right motor proportions are set now
  }

   else //set motor proportions based on ultrasonic input
   {
     //TUNNEL
      setMotorProportions(leftMotorProportion, rightMotorProportion);    
   }

  // // ULTRASONIC
  // long currMillis = millis();
  // if(currMillis - prevMillis >= uSonicInterval) {
  //   int dist = sonar.ping_cm();
  //   if(dist <= 8) {
  //     // Something is within 7 cm
  //     blockState = 1;
  //   } else if(dist <= 4) {
  //     // Something is within 4 cm
  //     blockState = 2;
  //   } else {
  //     blockState = 0;
  //   }
  // }

  // switch(blockState) {
  //   case 0:
  //     break;  

  // ULTRASONIC
  long currMillis = millis();
  if(currMillis - prevMillis >= uSonicInterval) {
    int dist = sonarBlock.ping_cm();
    if(dist <= 8) {
      // Something is within 7 cm
      blockState = 1;
    } else if(dist <= 4) {
      // Something is within 4 cm
      blockState = 2;
    } else {
      blockState = 0;
    }
  }

  // switch(blockState) {
  //   case 0:
  //     break;
    
  //   case 1:
  //     leftMotorProportion /= slowRatio;
  //     rightMotorProportion /= slowRatio;
  //     break;

  //   case 2:
  //     // TODO: Replace with block-grabbing code
  //     leftMotorProportion = 0;
  //     rightMotorProportion = 0 ;
  //     detector.initialiseDetector();
  //     break;

  //   default:
  //     Serial.println("Invalid block state.");
  //     break;
  // }

  // COLOUR DETECTION
  colourSensorVal = analogRead(colourPinIn);
  colour = detector.detectColour(colourSensorVal);

  if(colour == Blue) { // Blue
    digitalWrite(bluePinOut, HIGH);
    digitalWrite(redPinOut, LOW);
    delay(1000);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
  } else if(colour == Red) { // Red
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, HIGH);
    delay(1000);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
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

  // BLINKY
  if(leftMotorProportion == 0 && rightMotorProportion == 0 && blinkyState == true) {
    digitalWrite(blinkyPin, LOW);
    blinkyState = false;
  } else if(!(leftMotorProportion == 0 && rightMotorProportion == 0) && blinkyState == false) {
    digitalWrite(blinkyPin, HIGH);
    blinkyState = true;
  }

  if(printCounter == 100) {
    //Serial.println(readingPrint);
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
  //leftMotor->setSpeed(0);
  //rightMotor->setSpeed(0);

  //Using the sign of the motor proportion to set the direction of motion for each wheel
  switch(leftSign)
  {
    case 0:
      leftMotor->run(FORWARD);
      break;

    case 1: 
      leftMotor->run(BACKWARD);
      break;

    default:
      leftMotor->run(RELEASE);
      break;

  }

  switch(rightSign)
  {
    case 0:
      rightMotor->run(FORWARD);
      break;

    case 1: 
      rightMotor->run(BACKWARD);
      break;

    default:
      rightMotor->run(RELEASE);
      break;

  }

}

void setMotorProportions(float& leftMotorProportion, float& rightMotorProportion)
{ 
  float distance = NO_ECHO;
  float desired_distance = 3.9; // in cm
  float kp = 0.01;

  float time = sonarTunnel.ping() * 1e-6; //time in seconds
  float speed = 34300; //speed of sound in cm/s
  distance = (speed*time)/2;
  
  int counter = 0;

  float error = distance - desired_distance;

  leftMotorProportion -= kp*error;
  rightMotorProportion += kp*error;

  // if(counter = 1000000000)
  // {
  //   Serial.println("Distance measured: ");
  //   Serial.println(distance);
  
  //   Serial.println("Error: ");
  //   Serial.println(error);  
  //   counter = 0;
  // }
  // else
  // {
  //   counter++;
  // }
  
}