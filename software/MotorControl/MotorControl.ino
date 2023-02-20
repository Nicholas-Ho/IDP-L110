#include "MotorControl.h"

#include <Adafruit_MotorShield.h>
#include "LineFollower.h"
#include "ColourDetector.h"
#include <NewPing.h>

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

const uint8_t maxPower = 240; // 255 is too much

//Assign line sensor pins (left to right)
const int linePins[4] = {4, 5, 6, 7};
int lineReadings[4] = {0, 0, 0, 0};

//Assign Tunnel Ultrasonic pins
const int triggerPinTun = 3;
const int echoPinTun = 2;
const int MAX_DISTANCE_T = 20; //in centimetres

//Tunnel Ultrasonic
bool inTunnel = false;
NewPing sonarTunnel(triggerPinTun, echoPinTun, MAX_DISTANCE_T);
float desiredDistance = 3.9; // in cm
void tunnelControl(float&, float&);

//Block Ultrasonic
const int trigPinB = 1;
const int echoPinB = 0;
const int maxDistB = 10; //In cm

NewPing sonarBlock(trigPinB, echoPinB, maxDistB);

int blockState = 0; // State of using ultrasonic to detect a block
const int slowRatio = 2; // Ratio to slow down the motors by if block is detected (eg if slowRatio is 3, slow down by a factor of 3)

//Block Ultrasonic intervals
const int uSonicInterval = 50; // In milliseconds
long prevMillis = 0;

// Pins for colour detection
const int colourPinIn = A1; // Analog In
const int bluePinOut = 8; // Digital Out
const int redPinOut = 9; // Digital Out

Colour colour = 0;
bool haveBlock = false;
int colourSensorVal = 0;

/*
Robot States:
0 --> Stop state (waiting for button press)
1 --> Startup sequence
2 --> Line Following
3 --> Returning home
4 --> Home (Stopped)
*/
int robotState = 0;

//Instantiate a controller object, passing in references to the left and right motor objects
LineFollower controller = LineFollower(leftMotorProportion, rightMotorProportion, inTunnel, desiredDistance, haveBlock, colour, robotState);

//Instantiate a colour detector object
ColourDetector detector = ColourDetector();

//Blinky pin
const int blinkyPin = 12;
bool blinkyState = false; // True if blinking

//Push Button
uint8_t buttonPressed = 1;
int buttonPin = 10;

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

  Serial.println("Ready.");

  //Read button input every 50ms
  while (robotState == 0)
  { 

    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);

    buttonPressed = digitalRead(buttonPin); //Break out of loop if we read LOW on buttonPin 
    if(buttonPressed == HIGH)
    {
      robotState = 1;
      break;
    }
    delay(50); 
  }

  Serial.println("Starting....");

  if(robotState == 1)
  {         
    //Run forward
    leftMotor-> setSpeed(200);
    rightMotor -> setSpeed(200);
    leftMotor -> run(FORWARD);
    rightMotor -> run(FORWARD);
    delay(4700);

    // Turn left
    turnLeftArduino();

    //Stop  
    leftMotor -> run(RELEASE);
    rightMotor -> run(RELEASE);
    robotState = 2;
  }
  Serial.println("Robot is running.");
}

int printCounter = 0;

void loop() 
{   
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
  
  if(robotState == 4) { // Reached home
    Serial.println("Home");
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    while(true); // Stop Robot
  }

  if(!inTunnel) //set motor proportions based on line sensor input
  {
    controller.control(lineReadings); //left and right motor proportions are set now
    // leftMotorProportion = 0;
    // rightMotorProportion = 0;
  }
   else //set motor proportions based on ultrasonic input
   {
     //TUNNEL
      tunnelControl(leftMotorProportion, rightMotorProportion);    
   }

  // ULTRASONIC PROXIMITY SENSOR
  long currMillis = millis();
  if(currMillis - prevMillis >= uSonicInterval) {
    if(!haveBlock) {
      int dist = sonarBlock.ping_cm();    
      if(dist == 0) {
        // Nothing
      } else if(dist <= 3) {
        // Something is within 3 cm
        detector.initialiseDetector(); // Initialise colour detection algorithm
        leftMotorProportion /= slowRatio;
        rightMotorProportion /= slowRatio;
      } else if(dist <= 6) {
        // Something is within 6 cm
        leftMotorProportion /= slowRatio;
        rightMotorProportion /= slowRatio;
      }
      prevMillis = currMillis;
    }
  }

  // COLOUR DETECTION
  colourSensorVal = analogRead(colourPinIn);

  if(!haveBlock) {
    colour = detector.detectColour(colourSensorVal); // Call the colour detection algorithm
    if(colour == Blue || colour == Red) {
      displayColour(colour); // Stop and display the colour
      turnAroundArduino();
      moveStraightArduino(75, 500);
      controller.initialiseReturn();
    }
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
  // Provide power to the timer-LED if the motors are spinning
  if(leftMotorProportion == 0 && rightMotorProportion == 0 && blinkyState == true) {
    digitalWrite(blinkyPin, LOW);
    blinkyState = false;
  } else if(!(leftMotorProportion == 0 && rightMotorProportion == 0) && blinkyState == false) {
    digitalWrite(blinkyPin, HIGH);
    blinkyState = true;
  }

  // Debugging (most useful bit of code the entire project)
  if(printCounter == 100) {
    // Serial.println(readingPrint);
    printCounter = 0;
  } else {
    printCounter++;    
  }
  
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

// ULTRASONIC TUNNEL CONTROL
void tunnelControl(float& leftMotorProportion, float& rightMotorProportion)
{ 
  const static int interval = 100;
  static long tunnelMillis = 0;

  // Check if the line has been found again. If counter exceeds counter_max, line has been found
  static int counter = 0;                                                                                                       
  const static int counter_max = 100;

  const float basePower = 0.75;
  
  float distance = NO_ECHO;
  float kp = 0.2; // Proportional gain

  if(millis() - tunnelMillis >= interval) {
    distance = getTunnelDistance();
    
    int counter = 0;

    float error = distance - desiredDistance;

    // Proportional control
    leftMotorProportion = basePower;
    rightMotorProportion = basePower;
    leftMotorProportion -= kp*error;
    rightMotorProportion += kp*error;

    tunnelMillis = millis();
  }

  // Check if the line has been found
  int lineReadingSum = 0;
  for(int i=0; i<4; i++) {
    lineReadingSum += lineReadings[i];
  }
  if(lineReadingSum > 0) {
    counter++;
  } else {
    counter = 0;
  }
  if(counter >= counter_max) { // Line readings have been nonzero for 100 cycles
    Serial.println("Out of tunnel");
    inTunnel = false;
    counter = 0;
  } 
}

// Utility function for getting distance from the side wall (from side-mounted ultrasonic)
float getTunnelDistance() {
  float distance = NO_ECHO;
  float time = sonarTunnel.ping() * 1e-6; //time in seconds
  float speed = 34300; //speed of sound in cm/s
  distance = (speed*time)/2;

  return distance;
}

void turnLeftArduino() {
  //Turn left 90 degrees
  leftMotor-> setSpeed(150);
  rightMotor -> setSpeed(150);
  leftMotor -> run(BACKWARD);
  rightMotor -> run(FORWARD);
  delay(2600);
}

void turnRightArduino() {
  //Turn right 90 degrees
  leftMotor-> setSpeed(150);
  rightMotor -> setSpeed(150);
  leftMotor -> run(FORWARD);
  rightMotor -> run(BACKWARD);
  delay(2600);
}

void turnAroundArduino() {
  //Turn 180 degrees
  leftMotor-> setSpeed(150);
  rightMotor -> setSpeed(150);
  leftMotor -> run(FORWARD);
  rightMotor -> run(BACKWARD);
  delay(5600);
}

void moveStraightArduino(int speed, int delayTime) {
  //Move straight
  //75 speed for probing
  //150 speed otherwise
  leftMotor-> setSpeed(speed);
  rightMotor -> setSpeed(speed);
  leftMotor -> run(FORWARD);
  rightMotor -> run(FORWARD);
  delay(delayTime);
}

void reverseArduino(int speed, int delayTime)
{
  //Reverse
  leftMotor -> setSpeed(speed);
  rightMotor->setSpeed(speed);
  leftMotor -> run(BACKWARD);
  rightMotor -> run(BACKWARD);
  delay(delayTime);
  
}

// Display detected colour
void displayColour(Colour col) {
  haveBlock = true;

  if(col == Blue) {
    // Display coloured LED
    digitalWrite(bluePinOut, HIGH);
    digitalWrite(redPinOut, LOW);
  } else if(col == Red) {
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, HIGH);
  }

  // Stop movement
  leftMotor -> run(RELEASE);
  rightMotor -> run(RELEASE);

  // Stop blinky
  digitalWrite(blinkyPin, LOW);
  blinkyState = false;

  delay(6000);


  digitalWrite(bluePinOut, LOW);
  digitalWrite(redPinOut, LOW);
  digitalWrite(blinkyPin, HIGH);
  blinkyState = true;
}