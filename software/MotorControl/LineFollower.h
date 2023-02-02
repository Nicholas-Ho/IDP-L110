#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
#include "Arduino.h"

class LineFollower {

    public:
        LineFollower(float &leftM, float &rightM) : leftMotor(leftM), rightMotor(rightM) {};
        int control(int lineReadings[4]);
        int turnLeft(int lineBinary);

    private:
        float& leftMotor;
        float& rightMotor;

        float kp = 0.233; // In proportion of maximum power
        float basePower = 0.5; // Base power (before correction)

        int (LineFollower::*activeFunc)(int) = nullptr; // If there is an active function, skip main logic and call active function

        int detectEnd(int lineBinary);
        int detectJunction(int lineBinary);
        int followLine(int lineBinary);

        int turnLeft(int lineBinary);
        int turnRight(int lineBinary);
        int turnAround(int lineBinary);
        int moveStraight(int lineBinary);
        int probeJunction(int lineBinary);
        int pathfind(direction dir);

        float getTurningTime(float angle);

        int printCounter = 0;

};

#endif