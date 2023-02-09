#include "LineFollower.h"
#include "MotorControl.h"
#include <Arduino.h>

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
                Serial.println(size);
                return 0;
            } else {
                return -1;
            }
        }

        direction pop() {
            if(size > 0) {
                size--;
                Serial.println(size);
                return stack[size];
            } else {
                return ERROR_D;
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

int LineFollower::control(int lineReadings[4]) {
    // Convert to binary representations
    int lineBinary = lineReadings[0]*8 + lineReadings[1]*4 + lineReadings[2]*2 + lineReadings[3];
    if(activeFunc==nullptr) {
        int res = -1;
        res = detectEnd(lineBinary);
        if(res == 0) {return 0;}
        res = detectJunction(lineBinary);
        if(res == 0) {return 0;}
        res = followLine(lineBinary);
        return res;
    } else {
        (this->*activeFunc)(lineBinary);
    }
    return -1;
}

int LineFollower::pathfind()
{   //Function updates branchCounter and returns -1 if the turn is not to be taken, and 0 if it is
    /*
    If blockColour = -1, count up for each junction we see
    If blockColour = 0. Count down for each branch seen, deliver at branchCount = 2
    If blockColour = 1, Count down for each branch seen, deliver at branchCount = 0
    */

    int res = -1;

    switch(blockColour)
    {
        case -1:
            branchCounter++;
            break;
        case 0:
            branchCounter--;
            if(branchCounter == 2) {res = 0;} //Green delivery area
            break;
        case 1:
            branchCounter--;
            if(branchCounter == 0) {res = 0;} //Red delivery area
            break;
    }
    // Serial.println(branchCounter);

    return res;
}

int LineFollower::detectEnd(int lineBinary) {
    if(lineBinary == 0) {
      activeFunc = &probeEnd;
      return 0;
    }

    return -1;
}

int LineFollower::detectJunction(int lineBinary) {
    // Note: When turning 90 degrees, turn then move forward for a second to prevent the branch the robot
    // came from interferring
    static Stack dirStack = Stack();
    int pathfindRes; //Stores result of pathfind: 0 corresponds to turning and -1 to carrying on

    if(lineBinary == 15) { // [1 1 1 1]
        direction nextDir = dirStack.pop();
        // Serial.println(nextDir);
        if(nextDir == left) {
            // Turn 90 degrees to the left
            activeFunc = &turnLeft;
        } else if(nextDir == right) {
            // Turn 90 degrees to the right
            activeFunc = &turnRight;
        } else if(nextDir == straight) {
            // Move straight for a second (suspend control!)
            activeFunc = &moveStraight;
        } else {
            pathfindRes = pathfind();
            activeFunc = &moveStraight;
        }
    } else if(lineBinary == 14) { // [1 1 1 0]

        pathfindRes = pathfind(); //On left junction, we only want to count up or down, we never need to explore
        activeFunc = &moveStraight;

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
            pathfindRes = pathfind();
            if(pathfindRes == 0) 
            {
                activeFunc = &turnRight; //Turn right into the delivery area
                // TODO: Add block delivery function
                dirStack.add(left);
            }

            return 0;
        }
    
    }
    return -1;
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

    // Based on the error, do some proportional control
    leftMotor -= kp * error;
    rightMotor += kp * error;

    // Ensure that we're not trying to power the motors beyond their maximum
    leftMotor = max((float)-1, min((float)1, leftMotor));
    rightMotor = max((float)-1, min((float)1, rightMotor));

    return 0;
}

int LineFollower::turnLeft(int _) {
    moveStraightArduino(500);
    turnLeftArduino();
    moveStraightArduino(500);
    activeFunc = nullptr;
    return 0;
}

int LineFollower::turnRight(int _) {
    moveStraightArduino(500);
    turnRightArduino();
    moveStraightArduino(500);
    activeFunc = nullptr;
    return 0;
}

int LineFollower::turnAround(int _) {
    turnAroundArduino();
    activeFunc = nullptr;
    return 0;
}

int LineFollower::moveStraight(int _) {
    moveStraightArduino(2000);
    activeFunc = nullptr;
}

int LineFollower::probeJunction(int lineBinary) {
    static int count = 0;
    static const int max_count = 200; // TODO: Tune the duration of the probe

    if(count == max_count) {
        count = 0;
        if(probeStateJ == left) {
            activeFunc = &turnLeft;            
        } else if(probeStateJ == right) {
            activeFunc = &turnRight;
        } else {
            Serial.println("Error: Junction detected, but no probe state.");
            activeFunc = nullptr;
        }
        pathfind();
        probeStateJ = NONE_D;
        return 0;      
    } else if(lineBinary == 15) {
        count = 0;
        activeFunc = nullptr;
        probeStateJ = NONE_D;
        return 0;
    } else if((probeStateJ == left && lineBinary != 14) || (probeStateJ == right && lineBinary != 7)) {
        // False alarm, continue
        probeStateJ = NONE_D;
        activeFunc = nullptr;
        return 0;
    } else {
        leftMotor = basePower / 1.5;
        rightMotor = basePower / 1.5;
        count++;
        return 0;
    }
}

int LineFollower::probeEnd(int lineBinary) { 
    static int count = 0;
    static const int max_count = 200;

    if(count >= max_count) { // No line, end of branch
        count = 0;
        activeFunc = nullptr;
        turnAroundArduino();
        moveStraightArduino(500);
        return 0;
    } else if(lineBinary != 0) {
        count = 0;
        activeFunc = nullptr; // Resume normal function
        return 0;
    } else {
        leftMotor = basePower / 2;
        rightMotor = basePower / 2;

        float uSonicDist = getTunnelDistance();
        if(uSonicDist != 0.0 && uSonicDist < 6.0) { // In tunnel!
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