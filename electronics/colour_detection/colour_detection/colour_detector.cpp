#include "colour_detector.h"

void ColourDetector::initialiseDetector() {
  if(state == 0) { // Ensure no interruptions
    Serial.println("Initialising...");
    state = 1;
  }
}

// return 1: Blue
// return 2: Red
// return 0: No result (yet)
// return -1: ERROR
int ColourDetector::detectColour(int sensorVal) {
  currMillis = millis();

  if(currMillis - prevMillis >= interval) {
    switch(state) {
      case 0: // State 0: Dormant
        break;
      case 1: // State 1: Initialised
        pastSamples.resetFill(1023);
        state = 2;
        counter = 0;
        Serial.println("Recording");
        break;
      case 2: // State 2: Recording
        int prevVal = pastSamples.pop();

        // Serial.print(sensorVal);
        // Serial.print(" ");
        // Serial.println(prevVal);

        pastSamples.add(sensorVal);

        if(sensorVal - prevVal >= blueThreshold) {
          Serial.println("Blue!");
          state = 0;
          counter = 0; // Reset counter (just in case)
          return 1;
        } else {
          counter++;
          if(counter % 100 == 0) {
            Serial.print(counter/100);
            Serial.println(" seconds");
          }
          if(counter >= counterThresh) {
            Serial.println("Red!");
            state = 0;
            counter = 0; // Reset counter (just in case)
            return 2;
          }
        }
        break;
      default:
        Serial.println("Illegal state encountered.");
        return -1;
        break;
    }
    prevMillis = currMillis;
  }
  return 0;
}

int ColourDetector::getState() { // Just in case
    return state;
}