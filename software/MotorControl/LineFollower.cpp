#include "LineFollower.h"
#include "MotorControl.h"

// Public function. To be called by main .ino file
// Controls all logic in the LineFollower object
int LineFollower::control(int lineReadings[4])
{
  // Convert to binary representations
  int lineBinary = lineReadings[0] * 8 + lineReadings[1] * 4 + lineReadings[2] * 2 + lineReadings[3];
  if (activeFunc == nullptr) // If there is no active function, follow the main logic
  {
    int res = -1;
    res = detectEnd(lineBinary);
    if (res == 0)
    {
      return 0;
    }
    res = detectJunction(lineBinary);
    if (res == 0)
    {
      return 0;
    }
    res = followLine(lineBinary);
    return res;
  }
  else
  {
    (this->*activeFunc)(lineBinary); // If there is an active function, call it
  }
  return -1;
}

// Pathfinding function
// Keeps track of number of branches passed by the robot
// Counts up in no block, counts down if block is found and to be delivered
// Function updates branchCounter and returns -1 if the turn is not to be taken, and 0 if it is
int LineFollower::pathfind()
{
  int res = -1;

  if (!haveBlock && robotState == 2)
  {
    branchCounter++;
  }
  else
  {
    branchCounter--;
    switch (colour)
    {
    case Blue:
      if (branchCounter == 2)
      {
        res = 0;
      } // Green delivery area
      break;
    case Red:
      if (branchCounter == 0)
      {
        res = 0;
      } // Red delivery area
      break;
    default:
      if(robotState == 3 && branchCounter == 1) //Indicates that we should turn into the home delivery box
      {
        return 0;
      }
    }
  }

  Serial.println("Branch Counter: ");
  Serial.println(branchCounter);
  return res;
}

// Logic for dealing with the end of lines
// If there is no line detected, initiate check for tunnel and probe further to ensure no erronous readings
int LineFollower::detectEnd(int lineBinary)
{
  if (lineBinary == 0)
  {
    activeFunc = &checkTunnel;
    return 0;
  }

  return -1;
}

// Logic for dealing with junctions
int LineFollower::detectJunction(int lineBinary)
{
  // Note: When turning 90 degrees, turn then move forward for a second to prevent the branch the robot
  // came from from interferring
  int pathfindRes; // Stores result of pathfind: 0 corresponds to turning and -1 to carrying on

  if (lineBinary == 15)
  { // [1 1 1 1]. Two-branch junction
    // If haveBlock == true, reverse the stack (left becomes right and vice versa) to return home
    if (haveBlock && (colour == Red || colour == Blue))
    {
      dirStack.reverseStack();
    }

    // If not, check if there are any directions on the stack and execute if any
    direction nextDir = dirStack.pop();

    if (nextDir == left)
    {
      // Turn 90 degrees to the left
      activeFunc = &turnLeft;
    }
    else if (nextDir == right)
    {
      // Turn 90 degrees to the right
      activeFunc = &turnRight;
    }
    else if (nextDir == straight)
    {
      // Move straight for a second (suspend control!)
      activeFunc = &moveStraight;
    }
    else
    {
      // If there are no directions on the stack, simply skip the junction
      activeFunc = &moveStraight;
    }
  }
  else if (lineBinary == 14)
  { // [1 1 1 0]
    // On left junction, we only want to count up or down, we never need to explore (apart from returning home)
    // Call logic to confirm the junction's nature and execute respective logic
    activeFunc = &probeJunction;
    probeStateJ = left;

    return 0;
  }
  else if (lineBinary == 7)
  { // [0 1 1 1]
    // Call logic to confirm the junction's nature and execute respective logic
    activeFunc = &probeJunction;
    probeStateJ = right;

    return 0;
  }
  return -1;
}

// Core line following logic
// Implements proportional and derivative control
int LineFollower::followLine(int lineBinary)
{
  static int lastError = 0;
  int bufferSize = 20;
  static CircularBuffer previousErrors = CircularBuffer(bufferSize, true, 0); //For derivative control
  const float sampleDelay = 50; //50 ms delay between error readings
  static long startTime = millis();
  static int previousError = 0;

  // At base, move forward at desired power
  leftMotor = basePower;
  rightMotor = basePower;

  // If ideal state of [0 1 1 0], do nothing
  // If not, direction correction

  // Left is positive error, right is negative error
  int error;
  switch (lineBinary)
  {
  case 6: // [0 1 1 0]
    error = 0;
    break;
  case 1: // [0 0 0 1]
    error = -3;
    break;
  case 3: // [0 0 1 1]
    error = -2;
    break;
  case 2: // [0 0 1 0]
    error = -1;
    break;
  case 4: // [0 1 0 0]
    error = 1;
    break;
  case 12: // [1 1 0 0]
    error = 2;
    break;
  case 8: // [1 0 0 0]
    error = 3;
    break;
  default:
    // Default to the last error if there is an unexpected input (ie if it is turning left it will keep turning)
    error = lastError;
    break;
  }
  // Note: case [0 0 0 0] is not handled in the line following algorithm (branch end detection)
  // Note: cases [1 1 1 0] and [0 1 1 1] are not handled in the line following algorithm (junction detection)

  // Update lastError for switch default
  lastError = error;

  //Every 50 ms, get a new previous error for approximate derivative control
  if(millis() >= startTime + sampleDelay)
  {
    previousErrors.add(error); //Adding error to circular buffer
    previousError = previousErrors.pop();
    startTime = millis(); //reset start time
  }

  // Based on the error, do some proportional and derivative control
  leftMotor -= kp * error + kd * ((error - previousError)/sampleDelay);
  rightMotor += kp * error + kd * ((error - previousError)/sampleDelay);

  // Ensure that we're not trying to power the motors beyond their maximum
  leftMotor = max((float)-1, min((float)1, leftMotor));
  rightMotor = max((float)-1, min((float)1, rightMotor));

  return 0;
}

int LineFollower::initialiseReturn() {
  Serial.println("Delivering block.");
  if(dirStack.isEmpty()) {
    Serial.println("On the main line, incrementing branch counter.");
    branchCounter++; // Solving discrepancy between collecting the block down a branch and on the main line
  }
}

// ACTIVE FUNCTIONS

int LineFollower::turnLeft(int _)
{
  moveStraightArduino(75, 500);
  turnLeftArduino();
  moveStraightArduino(75, 500);
  activeFunc = nullptr;
  return 0;
}

int LineFollower::turnRight(int _)
{
  moveStraightArduino(75, 500);
  turnRightArduino();
  moveStraightArduino(75, 500);
  activeFunc = nullptr;
  return 0;
}

int LineFollower::turnAround(int _)
{
  turnAroundArduino();
  activeFunc = nullptr;
  return 0;
}

int LineFollower::moveStraight(int _)
{
  moveStraightArduino(150, 1000);
  activeFunc = nullptr;
}

int LineFollower::reverse(int _)
{
  reverseArduino(75, 5000);
  turnRightArduino();
  activeFunc = nullptr;
  return 0;
}

// NOTE: Only called for one-branch junctions
// Ensure that the one-branch junction is what it is initially detected to be by probing it for a set time
// If the junction turns out to be a two-branch junction or an erronous reading, break out of probe
// If not, implement logic:
// 1. Turn right on a right branch
// 2. Skip a left branch (unless otherwise specified on return to the starting box)
// 3. If robotState == 3, return if the junction is the starting box
int LineFollower::probeJunction(int lineBinary)
{
  static int count = 0;
  static const int maxCount = 70;

  if (count == maxCount) // If the junction is as initially detected, execute logic
  {
    count = 0;
    int pathfindRes = -1;
    if (probeStateJ == left)
    {
      pathfindRes = pathfind();
      if(robotState == 3 && pathfindRes == 0) // If the robot is returning home and this is the starting box, execute return sequence
      {
        activeFunc = &returnHome;
        return 0; // Don't want to reset probeStateJ;
      }
      activeFunc = &moveStraight; // On left junction, we only want to count up or down, we never need to explore
    }
    else if (probeStateJ == right)
    {
      pathfindRes = pathfind();
      if(robotState == 3 && pathfindRes == 0) // If the robot is returning home and this is the starting box, execute return sequence
      {
        activeFunc = &returnHome;
        return 0; // Don't want to reset probeStateJ;
      }
      else if (!haveBlock)
      {
        activeFunc = &turnRight;
        dirStack.add(right); // To ensure that the robot can continue down the main line when it turns around at the end of the branch
      }
      else if (haveBlock && pathfindRes == 0)
      {
        activeFunc = &deliverBlock; // Deliver block
      }
      else
      {
        // haveBlock but not the delivery area
        activeFunc = &moveStraight;
      }
    }
    else
    {
      Serial.println("Error: Junction detected, but no probe state.");
      activeFunc = nullptr;
    }
    probeStateJ = NONE_D;
    return 0;
  }
  else if (lineBinary == 15)
  { // If the junction turns out to be a two-branch junction, break out of probe and return to main logic
    count = 0;
    activeFunc = nullptr;
    probeStateJ = NONE_D;
    return 0;
  }
  else if ((probeStateJ == left && lineBinary != 14) || (probeStateJ == right && lineBinary != 7))
  { // If junction is not what it was initially detected to be, break out of probe and return to main logic
    // False alarm, continue
    probeStateJ = NONE_D;
    activeFunc = nullptr;
    return 0;
  }
  else
  {
    // Slow down during the probe
    leftMotor = 0.33;
    rightMotor = 0.33;
    count++;
    return 0;
  }
}

// At the end of the line, check if the robot is in the tunnel via side-mounted ultrasonic sensor
// If tunnel is detected, switch to ultrasonic tunnel control algorithm
// If not, call probeSweep() and check if the end of the line is truly the end of the line
int LineFollower::checkTunnel(int _) {
  float uSonicDist = getTunnelDistance();
    if (uSonicDist != 0.0 && uSonicDist < 10.0)
    { // In tunnel!
      inTunnel = true;
      Serial.println("Entering tunnel");
      desiredDistance = uSonicDist;            
      activeFunc = nullptr;
      return 0;
    } else {
      activeFunc = &probeSweep;
    }
    return -1;
}

// Simple sweep to ensure that an end of the line is truly the end of the line and not an erronous reading
// eg. Check whether the line somehow fit in between line sensors such that the reading is [0 0 0 0]
int LineFollower::probeSweep(int lineBinary)
{

  /*
  Sweep State 0 -> Sweep to the left
  Sweep State 1 -> Sweep back to the right
  */
  // State of the sweeping process
  static int sweepState = 0;
  // Counter for the number of cycles the sweep has happened
  static int count = 0;
  // Maximum count
  const int maxCount = 100;
  //Number of 1's seen on line
  static int lineCount = 0;
  const int maxLineCount = 50;

  if(sweepState == 0) 
  {
    if(count == maxCount)
    {
      count = 0;
      sweepState = 1;

    }
    else
    {
      count++;
      if(lineBinary)  {lineCount++;}
      if(lineCount > maxLineCount)
      {
        activeFunc = nullptr;
        return 0;
      }
      leftMotor = -basePower;
      rightMotor = basePower;
    }    
  }

  else if(sweepState == 1)
  {
    if(count == maxCount)
    {
      count = 0;
      sweepState = 0;

      if(lineCount > maxLineCount)
      {
        activeFunc = nullptr;
      }
      else 
      {
        activeFunc = &turnAround;
      }

      lineCount = 0;
      return 0;
    }
    else 
    {
      count++;
      if(lineBinary)  {lineCount++;}
      leftMotor = basePower;
      rightMotor = -basePower;
    }    

  }

  return 0; 

}

// Block delivery sequence
// Resets variables to initial state and initialises return to starting box
int LineFollower::deliverBlock(int _)
{
  turnRightArduino();
  moveStraightArduino(150, 3000);
  reverseArduino(150, 3000);
  if(colour == Red) {
    branchCounter += 2;
    turnRightArduino();
  } else {
    turnLeftArduino();
  }
  moveStraightArduino(90, 1000);
  haveBlock = false;
  colour = NONE_C;  
  activeFunc = nullptr;
  robotState = 3;
  return 0;
}

// Sequence for returning to the starting box
int LineFollower::returnHome(int _)
{
  if(probeStateJ == left) {
    turnLeftArduino();
  } else {
    turnRightArduino();
  }
  moveStraightArduino(150, 6000);
  robotState = 4;
  activeFunc = nullptr;
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation of helper functions
bool Stack::isEmpty()
{
  if (size > 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

int Stack::add(direction dir)
{  
  if (size < max_size)
  {
    stack[size] = dir;
    size++;
    return 0;
  }
  else
  {
    return -1;
  }
}

direction Stack::pop()
{
  if (size > 0)
  {
    size--;
    return stack[size];
  }
  else
  {
    return ERROR_D;
  }
}

int Stack::reverseStack()
{
  // Swap left with right in stack
  for (int i = 0; i < size; i++)
  {
    if (stack[i] == left)
    {
      stack[i] = right;
    }
    else if (stack[i] == right)
    {
      stack[i] = left;
    }
  }
  return 0;
}

// Helper functions
float max(float a, float b)
{
  return a >= b ? a : b;
}

float min(float a, float b)
{
  return a <= b ? a : b;
}