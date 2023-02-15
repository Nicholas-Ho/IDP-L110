#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL

#include "ColourDetector.h"

void displayColour(Colour col);

void turnLeftArduino();
void turnRightArduino();
void turnAroundArduino();
void moveStraightArduino(int speed, int delayTime);
void reverseArduino(int speed, int delayTime);

float getTunnelDistance();

#endif