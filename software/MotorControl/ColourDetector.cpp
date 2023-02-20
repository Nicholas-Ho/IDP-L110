#include "ColourDetector.h"

// To ensure that the algorithm cannot be initialised when it is already initialised
void ColourDetector::initialiseDetector() {
  if(state == 0) { // Ensure no interruptions
    Serial.println("Initialising...");
    state = 1;
  }
}

Colour ColourDetector::detectColour(int sensorVal) {
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

        if(sensorVal - prevVal >= blueThreshold) { // If the current value exceeds the previous value by the threshold, block is blue
          Serial.println("Blue!");
          state = 0;
          counter = 0; // Reset counter (just in case)
          return Blue;
        } else {
          counter++;
          if(counter % 100 == 0) {
            Serial.print(counter/100);
            Serial.println(" seconds");
          }
          if(counter >= counterThresh) { // If the algorithm times out, the block is red
            Serial.println("Red!");
            state = 0;
            counter = 0; // Reset counter (just in case)
            return Red;
          }
        }
        break;
      default:
        Serial.println("Illegal state encountered.");
        return ERROR_C;
        break;
    }
    prevMillis = currMillis;
  }
  return NONE_C;
}

int ColourDetector::getState() { // Just in case
    return state;
}