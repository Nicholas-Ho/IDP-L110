#include "LineFollower.h"

int LineFollower::control(int lineReadings[4], float irReadings[2]) {
    // Convert to binary representations
    int lineBinary = lineReadings[0]*8 + lineReadings[1]*4 + lineReadings[2]*2 + lineReadings[3];
    int irBinary = irReadings[0]*2 + irReadings[1];

    int res = 0;
    res = detectEnd(lineBinary, irBinary);
    if(res == 1) {return 1;}
    res = detectJunction(lineBinary);
    if(res == 1) {return 1;}
    res = followLine(lineBinary);
    return res;
}

int LineFollower::detectEnd(int lineBinary, int irBinary) {

    if(lineBinary <= 0) { // [0 0 0 0]

        if(irBinary >= 0) {
            // TODO: Implement tunnel navigation
        }

        // Turn 180 degrees
        // During turn, suspend control
        return 1;
    }

    return 0;
}

int LineFollower::detectJunction(int lineBinary) {
    if(lineBinary == 14) { // [1 1 1 0]
        // Turn 90 degrees left
        // During turn, suspend control
        return 1;
    } else if(lineBinary == 7) { // [0 1 1 1]
        // Turn 90 degrees right
        // During turn, suspend control
        return 1;
    }

    return 0;
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

    return 1;
}