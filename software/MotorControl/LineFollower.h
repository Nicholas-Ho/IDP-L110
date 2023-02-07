#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
//#include "Arduino.h"

// For clarity
enum direction {straight, left, right, NONE, ERROR};

// Physical constants
const float wheelSpan; //Distance between wheels
const float wheelRadius;
const float wheelRPM = 10; //Currently set to 10 RPM
const float wheelAngularSpeed = (wheelRPM*2*3.14/60);

class LineFollower {

    public:
        LineFollower(float &leftM, float &rightM) : leftMotor(leftM), rightMotor(rightM) {};
        int control(int lineReadings[4]);

    private:
        float& leftMotor;
        float& rightMotor;

        const float kp = 0.233; // In proportion of maximum power
        const float basePower = 0.5; // Base power (before correction)

        direction probeStateJ = NONE; // State for probe junction
        int branchCounter = 0; //Counting branches
        int blockColour = -1; //Keeping track of block colour + whether or not it is picked up

        int (LineFollower::*activeFunc)(int) = nullptr; // If there is an active function, skip main logic and call active function

        int detectEnd(int lineBinary);
        int detectJunction(int lineBinary);
        int followLine(int lineBinary);

        int turnLeft(int lineBinary);
        int turnRight(int lineBinary);
        int turnAround(int lineBinary);
        int moveStraight(int lineBinary);
        int probeJunction(int lineBinary);
        int probeEnd(int lineBinary);
        int pathfind();

        float getTurningTime(float angle);

        int printCounter = 0;

};

#endif