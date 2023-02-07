#include "circular_buffer.h"
#include <Arduino.h>

class ColourDetector {
    private:
        // State 0: Dormant
        // State 1: Initialising
        // State 2: Recording
        int state = 0;

        int blueThreshold = 30; // in bits

        // Checking against sample approximately 1 s ago
        CircularBuffer pastSamples = CircularBuffer(100, true, 1023);

        // Sample approximately every 10 ms
        int interval = 10;
        long currMillis = 0;
        long prevMillis = 0;

        // Counter, if approximately 5 seconds is up, declare red
        int counterThresh = 500;
        int counter = 0;

    public:
        void initialiseDetector();
        int detectColour(int sensorVal);
        int getState();
};