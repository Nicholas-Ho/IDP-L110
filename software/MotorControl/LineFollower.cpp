#include "LineFollower.h"
#include <Arduino.h>

int blockColour = -1; //Temp variable to store block colour. 0: Blue, 1: Red, -1: no block

// Helper stack function
class Stack {
    private:
        int size = 0;
        direction stack[3]; // Of size 3
        const int max_size = 3;

    public:
        bool isEmpty() {
            if(size > 0) {
                return false;
            } else {
                return true;
            }
        }

        int add(direction dir) {
            if(size < max_size) {
                stack[size] = dir;
                size++;
                return 0;
            } else {
                return -1;
            }
        }

        direction pop() {
            if(size > 0) {
                size--;
                return stack[size];
            } else {
                return ERROR;
            }
        }
};

//Helper functions
float max(float a, float b)
{
  return a >= b ? a : b;  
}

float min(float a, float b)
{
  return a <= b ? a : b;  
}

float LineFollower::getTurningTime(float angle)
{   //Calculating duration of turn
    float robotAngularSpeed = (2*wheelAngularSpeed*wheelRadius)/wheelSpan;
    float turningTime = angle/(robotAngularSpeed); //pi divided by angular speed
}

int LineFollower::control(int lineReadings[4]) {
    // Convert to binary representations
    int lineBinary = lineReadings[0]*8 + lineReadings[1]*4 + lineReadings[2]*2 + lineReadings[3];
    if(activeFunc==nullptr) {
        int res = -1;
        // res = detectEnd(lineBinary);
        if(res == 0) {return 0;}
        // res = detectJunction(lineBinary);
        if(res == 0) {return 0;}
        res = followLine(lineBinary);
        return res;
    } else {
        (this->*activeFunc)(lineBinary);
    }
    return -1;
}

int LineFollower::detectEnd(int lineBinary) {

    if(lineBinary <= 0) { // [0 0 0 0]
        // TODO: Add end detection leeway (probeEnd)

        // Turn 180 degrees
        // During turn, suspend control
        activeFunc = &turnAround;
        return 0;
    }

    return -1;
}

int LineFollower::pathfind(direction dir)
{   /*
    Function returns -1 if the robot should continue on, and 0 if it should turn into the home junction
    Add 1 for every branch on the right, take one away for every branch on the left
    Red delivery box -> 1
    Green delivery box -> 0
    */

    static int branchCounter = 0;
    int res;

    if(dir == right)
    { 
        switch(blockColour)
        {
            case -1:
                break;
            case 0: 
                if(branchCounter == -1) {res = 0;}
                break;
            case 1:
                if(branchCounter == 1) {res = 0;}
                break;          
        }
        branchCounter++;
    }
    else if (dir == left)
    {
        branchCounter--;
    }  

    return res;
}

int LineFollower::detectJunction(int lineBinary) {
    // Note: When turning 90 degrees, turn then move forward for a second to prevent the branch the robot
    // came from interferring
    static Stack dirStack = Stack();
    int pathfindRes; //Stores result of pathfind: 0 corresponds to turning and -1 to carrying on

    if(lineBinary == 15) { // [1 1 1 1]
        if(dirStack.isEmpty() == true) {
            // Skip cross-junction
            pathfindRes = pathfind(left); //Treating the two-way junction as a left junction in the pathfind
            activeFunc = &moveStraight;
        }

        direction nextDir = dirStack.pop();
        if(nextDir == left) {
            // Turn 90 degrees to the left
            activeFunc = &turnLeft;
        } else if(nextDir == right) {
            // Turn 90 degrees to the right
            activeFunc = &turnRight;
        } else if(nextDir == straight) {
            // Move straight for a second (suspend control!)
            activeFunc = &moveStraight;
        }
    } else if(lineBinary == 14) { // [1 1 1 0]

        if (blockColour == -1)  { //If the block has not been picked up, we want to explore the junction
            // Move forward a tiny bit to ensure that it isn't a two-way junction
            activeFunc = &probeJunction;
            probeStateJ = left;
            // If only one branch, turn 90 degrees to the left
            // If two branches, move to two-branch logic
            dirStack.add(left);
        }

        else    { //If block picked up, we want to pathfind to delivery zone
            pathfindRes = pathfind(left);
            //TODO: Make sure the junction isn't counted twice (move forward immediately)
        }

        return 0;

    } else if(lineBinary == 7) { // [0 1 1 1]
        if (blockColour == -1)
        {
            // Move forward a tiny bit to ensure that it isn't a two-way junction
            activeFunc = &probeJunction;
            probeStateJ = right;
            // If only one branch, turn 90 degrees to the right
            // If two branches, move to two-branch logic
            dirStack.add(right);
        }
        else    {
            tpathfindRes = pathfind(right);
            if(pathfindRes == 1) 
            {
                activeFunc = &turnRight; //Turn right into the delivery area
                // TODO: Add block delivery function
                dirStack.add(right);
            }

        return 0;
    }

    return -1;
  }
}

int LineFollower::followLine(int lineBinary) {
    static int lastError = 0;
    
    // At base, move forward at desired power
    leftMotor = basePower;
    rightMotor = basePower;

    // If ideal state of [0 1 1 0], do nothing
    // If not, direction correction

    // Left is positive error, right is negative error
    int error;
    switch(lineBinary) {
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

    if(printCounter == 100) {
      Serial.println(error);
      printCounter = 0;
    } else {
      printCounter++;    
    }

    // Based on the error, do some proportional control
    leftMotor -= kp * error;
    rightMotor += kp * error;

    // Ensure that we're not trying to power the motors beyond their maximum
    leftMotor = max((float)-1, min((float)1, leftMotor));
    rightMotor = max((float)-1, min((float)1, rightMotor));

    return 0;
}

int LineFollower::turnLeft(int _) {
    //Setting motors to turn left
    leftMotor = -basePower;
    rightMotor = basePower;

    float turningTime = getTurningTime(3.141/2);

    delay(turningTime*1000); //in milliseconds

    moveStraight(_);
    return 0;
}

int LineFollower::turnRight(int _) {

    //Setting motors to turn right
    leftMotor = basePower;
    rightMotor = -basePower;

    float turningTime = getTurningTime(3.141/2);

    delay(turningTime*1000); //in milliseconds
    
    moveStraight(_);
    return 0;
}

int LineFollower::turnAround(int _) {

    //Setting motors to turn left
    leftMotor = -basePower;
    rightMotor = basePower;

    float turningTime = getTurningTime(3.141);

    delay(turningTime*1000); //in milliseconds
    
    activeFunc = nullptr;
    return 0;
}

int LineFollower::moveStraight(int _) {
    //Setting motors to turn left
    leftMotor = basePower;
    rightMotor = basePower;

    delay(750); //in milliseconds. TODO: Tune duration
    
    activeFunc = nullptr;

    return 0;
}

int LineFollower::probeJunction(int lineBinary) {
    static int count = 0;
    static const int max_count = 100; // TODO: Tune the duration of the probe

    if(lineBinary == 15 || count == max_count) {
        count = 0;
        if(probeStateJ == left) {
            activeFunc = &turnLeft;
        } else if(probeStateJ == right) {
            activeFunc = &turnRight;
        } else {
            activeFunc = nullptr;
        }
        probeStateJ = NONE;
        return 0;
    } else {
        leftMotor = basePower / 3;
        rightMotor = basePower / 3;
        count++;
        return 0;
    }
}

int LineFollower::probeEnd(int lineBinary) {
    static int count = 0;
    static const int max_count = 100; // TODO: Tune the duration of the probe

    if(lineBinary != 0) { // If the line is detected again during the probe, continue following the line
        count = 0;
        activeFunc = nullptr;
        return 0;
    } else if(count == max_count) { // If not, turn around
        count = 0;
        activeFunc = &turnAround;
        return 0;
    } else {
        leftMotor = basePower / 3;
        rightMotor = basePower / 3;
        count++;
        return 0;
    }
}