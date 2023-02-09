#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL

#include "ColourDetector.h"

void displayColour(Colour col);

void turnLeftArduino();
void turnRightArduino();
void turnAroundArduino();
void moveStraightArduino(int delayTime);
void reverseArduino();

float getTunnelDistance();

#endif