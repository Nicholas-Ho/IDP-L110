#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

class LineFollower {

    public:
        LineFollower(float &leftM, float &rightM) : leftMotor(leftM), rightMotor(rightM) {};
        int control(int lineReadings[4]);

    private:
        float& leftMotor;
        float& rightMotor;

        float kp = 0.05; // In proportion of maximum power
        float basePower = 0.5; // Base power (before correction)

        int detectEnd(int readings[4]);
        int detectJunction(int readings[4]);
        int followLine(int readings[4]);

};

#endif