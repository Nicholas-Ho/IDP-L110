#include "LineFollower.h"
#include <Arduino.h>

// For T-junction logic
enum direction {straight, left, right, ERROR};

const float wheelSpan; //Distance between wheels
const float wheelRadius;

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

int LineFollower::detectEnd(int lineBinary) {

    if(lineBinary <= 0) { // [0 0 0 0]

        // Turn 180 degrees
        // During turn, suspend control
        activeFunc = &turnAround;
        return 0;
    }

    return -1;
}

int LineFollower::detectJunction(int lineBinary) {
    // Note: When turning 90 degrees, turn then move forward for a second to prevent the branch the robot
    // came from from interferring
    static Stack dirStack = Stack();

    if(lineBinary == 15) { // [1 1 1 1]
        if(dirStack.isEmpty() == true) {
            // Visit the left branch, the right branch then continue down the path
            dirStack.add(left);
            dirStack.add(straight);
            dirStack.add(right);
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
        // Move forward a tiny bit to ensure that it isn't a two-way junction
        activeFunc = &probeJunction;
        // If only one branch, turn 90 degrees to the left
        // If two branches, move to two-branch logic
        activeFunc = &turnLeft;
        dirStack.add(left);
        return 0;
    } else if(lineBinary == 7) { // [0 1 1 1]
        // Move forward a tiny bit to ensure that it isn't a two-way junction
        activeFunc = &probeJunction;
        // If only one branch, turn 90 degrees to the right
        // If two branches, move to two-branch logic
        activeFunc = &turnRight;
        dirStack.add(right);
        return 0;
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

    // //TODO: Interrupt control to get RPM from rotary encoder
    // float wheelAngularSpeed = 10;

    // //Calculating duration of 90 degree turn
    // float angularVelocity = (2*wheelAngularSpeed*wheelRadius)/wheelSpan;
    // float turningTime = 3.141/angularVelocity;

    // while ((millis() - start_time) < turningTime) {}

    moveStraight(_);
    return 0;
}

int LineFollower::turnRight(int _) {
    // TODO: Turn 90 degrees to the right
    moveStraight(_);
    return 0;
}

int LineFollower::turnAround(int _) {
    // TODO: Turn 180 degrees
    // Remove this
    activeFunc = nullptr;
    return 0;
}

int LineFollower::moveStraight(int _) {
    static int count = 0;
    static const int max_count = 10; // TODO: Tune the duration of the movement

    if(count == max_count) {
        count = 0;
        activeFunc = nullptr;
        return 0;
    } else {
        leftMotor = basePower;
        rightMotor = basePower;
        count++;
        return 0;
    }
}

int LineFollower::probeJunction(int lineBinary) {
    static int count = 0;
    static const int max_count = 10; // TODO: Tune the duration of the probe

    if(lineBinary == 15 || count == max_count) {
        count = 0;
        activeFunc = nullptr;
        return 0;
    } else {
        leftMotor = basePower / 3;
        rightMotor = basePower / 3;
        count++;
        return 0;
    }
}