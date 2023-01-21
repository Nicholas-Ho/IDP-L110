#include "LineFollower.h"
#include "MockRobot.h"
#include <string>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    std::string initial_x_s(argv[1]);
    float initial_x = std::stof(initial_x_s);
    std::string initial_y_s(argv[2]);
    float initial_y = std::stof(initial_y_s);
    std::string initial_th_s(argv[3]);
    float initial_th = std::stof(initial_th_s);

    std::ofstream OutFile("output.csv");

    MockRobot robot = MockRobot(initial_x, initial_y, initial_th);
    LineFollower controller = LineFollower(robot.leftMotor, robot.rightMotor);

    int max_time = 10; // Seconds

    float x;
    float y;
    for(int i=0; i < (max_time/TIMESTEP); i++) {
        robot.simulateMovement();
        robot.getSensorReadings();
        controller.control(robot.lineReadings, robot.irReadings);
        robot.calculateVelocity();

        robot.getPosition(x, y);
        OutFile << x << "," << y << "\n";
    }

    OutFile.close();
    return 0;
}

// int main() {
//     float leftMotor = 0;
//     float rightMotor = 0;
//     int mockData[10][4] = {
//         {0, 1, 1, 0},
//         {0, 1, 1, 0},
//         {0, 1, 0, 0},
//         {1, 1, 0, 0},
//         {0, 1, 0, 0},
//         {0, 1, 1, 0},
//         {0, 0, 1, 1},
//         {0, 0, 1, 0},
//         {0, 0, 0, 1},
//         {0, 1, 1, 0}
//     };

//     LineFollower controller = LineFollower(leftMotor, rightMotor);

//     for(int i=0; i < 10; i++) {
//         controller.control(mockData[i]);
//         std::cout << "Input: ";
//         for(int j=0; j < 4; j++) {
//             std::cout << mockData[i][j];
//         }
//         std::cout << "\nLeft Motor: " << leftMotor << ", Right Motor: " << rightMotor << "\n";
//     }

//     return 0;
// }