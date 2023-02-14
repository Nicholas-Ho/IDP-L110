#ifndef COLOUR_DETECTOR_H
#define COLOUR_DETECTOR_H

#include "circular_buffer.h"
#include <Arduino.h>

enum Colour {NONE_C, Blue, Red, ERROR_C};

class ColourDetector {
    private:
        // State 0: Dormant
        // State 1: Initialising
        // State 2: Recording
        int state = 0;

        int blueThreshold = 70; // in bits

        // Checking against sample approximately 0.5 s ago
        CircularBuffer pastSamples = CircularBuffer(50, true, 1023);

        // Sample approximately every 10 ms
        int interval = 10;
        long currMillis = 0;
        long prevMillis = 0;

        // Counter, if approximately 1.5 seconds is up, declare red
        int counterThresh = 100;
        int counter = 0;

    public:
        void initialiseDetector();
        Colour detectColour(int sensorVal);
        int getState();
};

#endif