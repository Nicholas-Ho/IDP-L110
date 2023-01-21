#define _USE_MATH_DEFINES
#include <cmath>
#include "MockRobot.h"
#include <iostream>

MockRobot::MockRobot(float x/*=0*/, float y/*=0*/, float angular_position/*=0*/) {
    pos_x = x;
    pos_y = y;
    angular_p = angular_position;
    // Calculate position of sensor relative to robot
    float s_pos = SENSOR_SPACING * 1.5;
    for(int i=0; i < 4; i++) {
        sensor_dist[i] = sqrt(pow(s_pos, 2) + pow(SENSOR_DISTANCE, 2));
        sensor_angle[i] = atan(s_pos/SENSOR_DISTANCE);
        s_pos -= SENSOR_SPACING;
    }
}

void MockRobot::getPosition(float &x, float &y) {
    x = pos_x;
    y = pos_y;
}

void MockRobot::simulateMovement() {
    // Update positions
    pos_x += velocity * cos(angular_p) * TIMESTEP;
    pos_y += velocity * sin(angular_p) * TIMESTEP;
    angular_p += angular_v * TIMESTEP;
}

void MockRobot::calculateVelocity() {
    float leftWheelV = leftMotor * max_ang_v * WHEEL_RADIUS;
    float rightWheelV = rightMotor * max_ang_v * WHEEL_RADIUS;
    velocity = (leftWheelV + rightWheelV) / 2;
    angular_v = (rightWheelV - leftWheelV) / (2 * ROBOT_RADIUS);
}

void MockRobot::getSensorReadings() {
    // Calculate sensor positions (just y will do)
    for(int i=0; i < 4; i++) {
        float angle = angular_p + sensor_angle[i];
        float sensor_y = pos_y + sensor_dist[i] * sin(angle);
        if(sensor_y <= (LINE_WIDTH/2) && sensor_y >= -(LINE_WIDTH/2)) {
            readings[i] = 1;
        } else {
            readings[i] = 0;
        }
    }
    // std::cout << readings[0] << readings[1] << readings[2] << readings[3] << "\n";
}