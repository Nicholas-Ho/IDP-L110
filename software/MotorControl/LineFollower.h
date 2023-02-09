#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
#include "Arduino.h"

// For clarity
enum direction {straight, left, right, NONE_D, ERROR_D};

// Physical constants
const float wheelSpan = 10; //Distance between wheels
const float wheelRadius = 10;
const float wheelRPM = 10; //Currently set to 10 RPM
const float wheelAngularSpeed = (wheelRPM*2*3.14/60);

// Helper stack class
class Stack {
    private:
        int size = 0;
        direction stack[3]; // Of size 3
        const int max_size = 3;

    public:
        bool isEmpty();
        int add(direction dir);
        direction pop();
        int reverseStack(); // Swap left with right in stack
};

//Helper functions
float max(float a, float b);
float min(float a, float b);



class LineFollower {

    public:
        LineFollower(float &leftM, float &rightM, bool& tunnel) : leftMotor(leftM), rightMotor(rightM), inTunnel(tunnel) {};
        int control(int lineReadings[4]);

    private:
        float& leftMotor;
        float& rightMotor;

        bool& inTunnel;

        const float kp = 0.3; // In proportion of maximum power
        const float basePower = 0.75; // Base power (before correction)

        Stack dirStack = Stack();
        direction probeStateJ = NONE_D; // State for probe junction
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