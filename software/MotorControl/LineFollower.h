#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
//#include "Arduino.h"
#include "ColourDetector.h"

// For clarity
enum direction {straight, left, right, NONE_D, ERROR_D};

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
        LineFollower(float &leftM, float &rightM, bool& tunnel, bool& block, Colour& col, int& state) : 
        leftMotor(leftM), rightMotor(rightM), inTunnel(tunnel), haveBlock(block), colour(col), robotState(state) {};
        int control(int lineReadings[4]);
        int returnHome();

    private:
        float& leftMotor;
        float& rightMotor;

        bool& inTunnel;
        bool& haveBlock;
        Colour& colour;
        int& robotState;

        const float kp = 0.3; // In proportion of maximum power
        const float basePower = 0.7; // Base power (before correction)

        Stack dirStack = Stack();
        direction probeStateJ = NONE_D; // State for probe junction

        int (LineFollower::*activeFunc)(int) = nullptr; // If there is an active function, skip main logic and call active function
        
        int pathfind();

        int detectEnd(int lineBinary);
        int detectJunction(int lineBinary);
        int followLine(int lineBinary);

        int turnLeft(int lineBinary);
        int turnLeftT(int lineBinary);
        int turnRight(int lineBinary);
        int turnRightT(int lineBinary);
        int turnAround(int lineBinary);
        int moveStraight(int lineBinary);
        int reverse(int lineBinary);
        int probeJunction(int lineBinary);
        int probeSweep(int lineBinary);
        int probeEnd(int lineBinary);
        int checkTunnel(int lineBinary);
        int deliverBlock(int lineBinary);

        int printCounter = 0;

};

#endif