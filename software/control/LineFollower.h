#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

class LineFollower {

    public:
        LineFollower(float &leftM, float &rightM) : leftMotor(leftM), rightMotor(rightM) {};
        int control(int lineReadings[4], float irReadings[2]);

    private:
        float& leftMotor;
        float& rightMotor;

        float kp = 0.05; // In proportion of maximum power
        float basePower = 0.5; // Base power (before correction)

        int detectEnd(int lineBinary, int irBinary);
        int detectJunction(int lineBinary);
        int followLine(int lineBinary);

};

#endif