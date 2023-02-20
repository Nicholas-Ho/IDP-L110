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


// Class implementing line following logic (and pathfinding)
class LineFollower {

    public:
        // Pulling references to important variables in the main .ino file
        LineFollower(float &leftM, float &rightM, bool& tunnel, float& desired_dist, bool& block, Colour& col, int& state) : 
          leftMotor(leftM), rightMotor(rightM), inTunnel(tunnel), desiredDistance(desired_dist), haveBlock(block), colour(col), robotState(state) {};
        int control(int lineReadings[4]); // The only method through which all logic in the class is called
        int initialiseReturn(); // Updating states during initialisation of the return to the starting box

    private:
        // References to the .ino motor proportion variables
        float& leftMotor;
        float& rightMotor;

        // References to other required variables in the .ino file
        bool& inTunnel;
        float& desiredDistance;
        bool& haveBlock;
        Colour& colour;
        int& robotState;

        int branchCounter = 1; // Keeps track of which junction the robot has passed

        // Motor control constants (gains and base power)
        const float kp = 0.25; // In proportion of maximum power
        const float kd = 10; // In proportion of maximum power
        const float basePower = 0.8; // Base power (before correction)

        Stack dirStack = Stack();
        direction probeStateJ = NONE_D; // State for probe junction

        int (LineFollower::*activeFunc)(int) = nullptr; // If there is an active function, skip main logic and call active function
        
        // Pathfinding method
        // Counts the number of junctions passed by the robot
        int pathfind();

        // Logic for dealing with end of lines, junctions and the default state of following the line
        // Called every time control() is called
        int detectEnd(int lineBinary);
        int detectJunction(int lineBinary);
        int followLine(int lineBinary);

        // ACTIVE FUNCTIONS
        // Functions to be assigned to activeFunc. If there is an active func, the main control logic will be skipped

        // Utility functions
        int turnLeft(int lineBinary);
        int turnRight(int lineBinary);
        int turnAround(int lineBinary);
        int moveStraight(int lineBinary);
        int reverse(int lineBinary);

        // Logic for dealing with end of lines, junctions and blocks. Called from detectEnd() or detectJunction()
        int probeJunction(int lineBinary);
        int probeSweep(int lineBinary);
        int checkTunnel(int lineBinary);
        int deliverBlock(int lineBinary);
        int returnHome(int lineBinary);

        int printCounter = 0;

};

#endif