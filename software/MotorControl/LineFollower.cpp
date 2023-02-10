#include "LineFollower.h"
#include "MotorControl.h"

int LineFollower::control(int lineReadings[4])
{
  // Convert to binary representations
  int lineBinary = lineReadings[0] * 8 + lineReadings[1] * 4 + lineReadings[2] * 2 + lineReadings[3];
  if (activeFunc == nullptr)
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
    (this->*activeFunc)(lineBinary);
  }
  return -1;
}

int LineFollower::pathfind()
{ // Function updates branchCounter and returns -1 if the turn is not to be taken, and 0 if it is

  static int branchCounter = 1;
  int res = -1;
  if (!haveBlock)
  {
    branchCounter++;
  }
  else
  {
    switch (colour)
    {
    case Blue:
      branchCounter--;
      if (branchCounter == 2)
      {
        res = 0;
      } // Green delivery area
      break;
    case Red:
      branchCounter--;
      if (branchCounter == 0)
      {
        res = 0;
      } // Red delivery area
      break;
    }
  }

  Serial.println("Branch Counter: ");
  Serial.println(branchCounter);
  return res;
}

int LineFollower::detectEnd(int lineBinary)
{
  if (lineBinary == 0)
  {
    activeFunc = &probeEnd;
    return 0;
  }

  return -1;
}

int LineFollower::detectJunction(int lineBinary)
{
  // Note: When turning 90 degrees, turn then move forward for a second to prevent the branch the robot
  // came from interferring
  int pathfindRes; // Stores result of pathfind: 0 corresponds to turning and -1 to carrying on

  if (lineBinary == 15)
  { // [1 1 1 1]
    // If haveBlock == true, reverse the stack (left becomes right and vice versa)
    if (haveBlock && (colour == Red || colour == Blue))
    {
      dirStack.reverseStack();
    }

    direction nextDir = dirStack.pop();
    Serial.println(nextDir);

    if (nextDir == left)
    {
      // Turn 90 degrees to the left
      activeFunc = &turnLeftT;
    }
    else if (nextDir == right)
    {
      // Turn 90 degrees to the right
      activeFunc = &turnRightT;
    }
    else if (nextDir == straight)
    {
      // Move straight for a second (suspend control!)
      activeFunc = &moveStraight;
    }
    else
    {
      activeFunc = &moveStraight;
    }
  }
  else if (lineBinary == 14)
  { // [1 1 1 0]

    pathfindRes = pathfind(); // On left junction, we only want to count up or down, we never need to explore
    activeFunc = &probeJunction;
    probeStateJ = left;

    return 0;
  }
  else if (lineBinary == 7)
  { // [0 1 1 1]
    // Move forward a tiny bit to ensure that it isn't a two-way junction
    activeFunc = &probeJunction;
    probeStateJ = right;
    // If only one branch, turn 90 degrees to the right
    // If two branches, move to two-branch logic

    return 0;
  }
  return -1;
}

int LineFollower::followLine(int lineBinary)
{
  static int lastError = 0;

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
    error = -2.5;
    break;
  case 3: // [0 0 1 1]
    error = -1.5;
    break;
  case 2: // [0 0 1 0]
    error = -1;
    break;
  case 4: // [0 1 0 0]
    error = 1;
    break;
  case 12: // [1 1 0 0]
    error = 1.5;
    break;
  case 8: // [1 0 0 0]
    error = 2.5;
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

  // Based on the error, do some proportional control
  leftMotor -= kp * error;
  rightMotor += kp * error;

  // Ensure that we're not trying to power the motors beyond their maximum
  leftMotor = max((float)-1, min((float)1, leftMotor));
  rightMotor = max((float)-1, min((float)1, rightMotor));

  return 0;
}

int LineFollower::returnHome()
{
  return -1;
}

int LineFollower::turnLeft(int _)
{
  moveStraightArduino(75, 500);
  turnLeftArduino();
  moveStraightArduino(75, 500);
  activeFunc = nullptr;
  return 0;
}

int LineFollower::turnLeftT(int _) {
  moveStraightArduino(75, 500);
  activeFunc = &turnLeft;
}

int LineFollower::turnRight(int _)
{
  moveStraightArduino(75, 500);
  turnRightArduino();
  moveStraightArduino(75, 500);
  activeFunc = nullptr;
  return 0;
}

int LineFollower::turnRightT(int _) {
  moveStraightArduino(75, 500);
  activeFunc = &turnRight;
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

int LineFollower::probeJunction(int lineBinary)
{
  static int count = 0;
  static const int max_count = 100;

  if (count == max_count)
  {
    count = 0;
    int pathfindRes = pathfind();
    if (probeStateJ == left)
    {
      activeFunc = &moveStraight; // On left junction, we only want to count up or down, we never need to explore
    }
    else if (probeStateJ == right)
    {
      if (!haveBlock)
      {
        activeFunc = &turnRight;
        dirStack.add(right);
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
  {
    count = 0;
    activeFunc = nullptr;
    probeStateJ = NONE_D;
    return 0;
  }
  else if ((probeStateJ == left && lineBinary != 14) || (probeStateJ == right && lineBinary != 7))
  {
    // False alarm, continue
    probeStateJ = NONE_D;
    activeFunc = nullptr;
    return 0;
  }
  else
  {
    leftMotor = 0.33;
    rightMotor = 0.33;
    count++;
    return 0;
  }
}

int LineFollower::probeEnd(int lineBinary)
{
  static int count = 0;
  static const int max_count = 150;

  if (count >= max_count)
  { // No line, end of branch
    // Do a sweep to check for false alarm
    count = 0;
    activeFunc = &probeSweep;

    return 0;
  }
  else if (lineBinary != 0)
  {
    count = 0;
    activeFunc = nullptr; // Resume normal function
    return 0;
  }
  else
  {
    leftMotor = 0.5;
    rightMotor = 0.5;

    float uSonicDist = getTunnelDistance();
    if (uSonicDist != 0.0 && uSonicDist < 6.0)
    { // In tunnel!
      inTunnel = true;
      count = 0;
      activeFunc = nullptr;
      Serial.println("Entering tunnel");
      return 0;
    }

    count++;
    return 0;
  }
}

int LineFollower::probeSweep(int lineBinary)
{
  static int sweepState = 0; //0 -> turn Left, 1 -> turnRight, 2 -> turnLeft again
  static int lineBinarySum = 0;
  static int count = 0;
  const int max_count_left = 50;
  const int max_count_right = 100;

  if (sweepState == 0 || sweepState == 2)
  {   
    if(count == max_count_left)  
    {
        count = 0;
        if(sweepState == 0)
        {
          if(lineBinarySum)  //if line detected at all, keep going and don't turn around
          {
            activeFunc = nullptr;
            lineBinarySum = 0;
            sweepState = 0; 
            return 0;
          }
          else {
            sweepState = 1;
            return 0;            
          }
        } 
        else  {
            if(lineBinarySum)  //if line detected at all, keep going and don't turn around
          {
            activeFunc = nullptr;
            lineBinarySum = 0;
            sweepState = 0; 
            return 0;
          } 
          else 
          { //sweep didn't detect a line, turn around
            activeFunc = nullptr;
            lineBinarySum = 0;
            sweepState = 0; 
            turnAroundArduino();
            moveStraightArduino(75, 500);
            return 0;
          }
        }
        }

    else
    {
      leftMotor = -basePower; //Sweeping to the left
      rightMotor = basePower;
      lineBinarySum += lineBinary;
      count++;
    }
    return 0;
  }
  else if (sweepState == 1)
  {
    if(count == max_count_right)  
    {
      if(lineBinarySum)
      {
        activeFunc = nullptr;
        sweepState = 0;
        lineBinary = 0;
      }
      else
      {
        count = 0;
        sweepState = 2;
      }
    }
    else
    {
      leftMotor = basePower; //Sweeping to the right
      rightMotor = -basePower;      
      lineBinarySum += lineBinary;
      count++;
    }
    return 0;
  }
}

int LineFollower::deliverBlock(int _)
{
  turnRightArduino();
  moveStraightArduino(150, 3000);
  reverseArduino(150, 3000);
  turnRightArduino();
  moveStraightArduino(75, 1000);
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