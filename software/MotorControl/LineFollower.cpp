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
  
  // if(robotState == 3 && branchCounter == 0) //Indicates that we should turn into the home delivery box
  // {
  //   return 0;
  // }

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
      activeFunc = &moveStraight;
    }
  }
  else if (lineBinary == 14)
  { // [1 1 1 0]

    // On left junction, we only want to count up or down, we never need to explore
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
  int bufferSize = 20;
  static CircularBuffer previousErrors = CircularBuffer(bufferSize, true, 0);
  const float sampleDelay = 500; //500 ms delay between error readings
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
  previousErrors.add(error); //Adding error to circular buffer

  //Every 500 ms, get a new previous error
  if(millis() >= startTime + sampleDelay)
  {
    previousError = previousErrors.pop();
    startTime = millis(); //reset start time
  }

  // Based on the error, do some proportional control
  leftMotor -= kp * error + kd * ((error - previousError)/sampleDelay);
  rightMotor += kp * error + kd * ((error - previousError)/sampleDelay);

  // Ensure that we're not trying to power the motors beyond their maximum
  leftMotor = max((float)-1, min((float)1, leftMotor));
  rightMotor = max((float)-1, min((float)1, rightMotor));

  return 0;
}

int LineFollower::returnHome()
{ 
  turnRightArduino();
  moveStraightArduino(150, 3000);
  robotState = 0; //Stop state
  
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

// int LineFollower::probeSweep(int lineBinary)
// {
//   // State of the sweeping process
//   static int sweepState = 0;
//   // Sum of the binary values for line detection
//   static int lineBinarySum = 0;
//   // Counter for the number of times the sweep has happened
//   static int count = 0;
//   // Maximum number of sweeps to the left
//   const int maxCountLeft = 50;
//   // Maximum number of sweeps to the right
//   const int maxCountRight = 100;

//   switch (sweepState)
//   {
//   case 0: // First sweep to the left
//   case 2: // Second sweep to the left
//     if (count == maxCountLeft)
//     {
//       count = 0;

//       if (lineBinarySum)
//       {
//         // If a line was detected, stop sweeping and reset binary sum
//         activeFunc = nullptr;
//         lineBinarySum = 0;
//         return 0;
//       }

//       // If no line was detected, go to the next sweep state
//       sweepState = (sweepState == 0) ? 1 : 0;
//       activeFunc = nullptr;
//       lineBinarySum = 0;
//       turnAroundArduino();
//       moveStraightArduino(75, 500);
//       return 0;
//     }

//     // Move the motors left or right depending on the sweep state
//     leftMotor = (sweepState == 0) ? -basePower : basePower;
//     rightMotor = (sweepState == 0) ? basePower : -basePower;
//     // Keep track of the binary sum
//     lineBinarySum += lineBinary;
//     // Increment the sweep count
//     count++;
//     break;

//   case 1: // Sweep to the right
//     if (count == maxCountRight)
//     {
//       if (lineBinarySum)
//       {
//         // If a line was detected, stop sweeping and reset binary sum
//         activeFunc = nullptr;
//         sweepState = 0;
//         lineBinarySum = 0;
//         return 0;
//       }

//       // If no line was detected, go back to the second sweep to the left
//       count = 0;
//       sweepState = 2;
//       break;
//     }

//     // Move the motors right
//     leftMotor = basePower;
//     rightMotor = -basePower;
//     // Keep track of the binary sum
//     lineBinarySum += lineBinary;
//     // Increment the sweep count
//     count++;
//     break;
//   }

//   activeFunc = &checkTunnel;

//   return 0;
// }


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
  static const int max_count = 50;

  if (count == max_count)
  {
    count = 0;
    int pathfindRes = -1;
    if (probeStateJ == left)
    {
      pathfindRes = pathfind();
      activeFunc = &moveStraight; // On left junction, we only want to count up or down, we never need to explore
      dirStack.add(left);
    }
    else if (probeStateJ == right)
    {
      pathfindRes = pathfind();
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
    count = 0;
    activeFunc = &checkTunnel;

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

    count++;
    return 0;
  }
}

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
      activeFunc = &turnAround;
    }
    return -1;
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