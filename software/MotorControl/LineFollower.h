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
        LineFollower(float &leftM, float &rightM, bool& tunnel, bool& block, Colour& col) : leftMotor(leftM), rightMotor(rightM), inTunnel(tunnel), 
        haveBlock(block), colour(col) {};
        int control(int lineReadings[4]);
        int returnHome();

    private:
        float& leftMotor;
        float& rightMotor;

        bool& inTunnel;
        bool& haveBlock;
        Colour& colour;

        const float kp = 0.33; // In proportion of maximum power
        const float basePower = 0.8; // Base power (before correction)

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
// <<<<<<< HEAD
//         int pathfind();
//         int sweep();

//         float getTurningTime(float angle);
// =======
        int deliverBlock(int lineBinary);
// >>>>>>> 33b9897e87a0e5bc4140c0ca197b37178a4cad8c

        int printCounter = 0;

};

#endif