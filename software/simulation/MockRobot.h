#ifndef MOCKROBOT_H
#define MOCKROBOT_H

#define _USE_MATH_DEFINES
#include <cmath>

// Constants for test suite (all in metres)
const float LINE_WIDTH = 0.019;
const float ROBOT_RADIUS = 0.07;
const float WHEEL_RADIUS = 0.02;
const float SENSOR_DISTANCE = 0.02; // In front of robot
const float SENSOR_SPACING[4] = {0.015, 0.009, 0.015, -1}; // Between sensors

const float TIMESTEP = 0.001; // Seconds

class MockRobot {
    public:
        float leftMotor = 0;
        float rightMotor = 0;
        int lineReadings[4] = {-1, -1, -1, -1};

        MockRobot(float x=0, float y=0, float angular_position=0);

        void simulateMovement();
        void calculateVelocity();
        void getSensorReadings();

        void getPosition(float &x, float &y);

    private:
        float pos_x;
        float pos_y;
        float angular_p; // In radians, direction of x-axis as 0
        float velocity = 0;
        float angular_v = 0; // In radians per second
        const float max_ang_v = 6 * M_PI; // Maximum angular velocity of the wheels. 3 revolutions per second

        float sensor_dist[4] = {0, 0, 0, 0};
        float sensor_angle[4] = {0, 0, 0, 0};
};

#endif