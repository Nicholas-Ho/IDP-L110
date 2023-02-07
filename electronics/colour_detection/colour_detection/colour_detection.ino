#include "circular_buffer.h"

// Analogue pin
int colourPinIn = A0;
int isBlue = 0;
int isRed = 0;

// Digital pin (for test button)
int buttonPin = 2;

int blueThreshold = 40; // in bits

// State 0: Paused
// State 1: Detecting
int state = 1;
int prevState = 1; // To detect change in state

// Checking against sample approximately 1 s ago
CircularBuffer pastSamples = CircularBuffer(100, true, 1023);

// Sample approximately every 10 ms
int interval = 10;
long currMillis = 0;
long prevMillis = 0;

// Counter, if approximately 3 seconds is up, declare red
int counterThresh = 300;
int counter = 0;

void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorVal = analogRead(colourPinIn);
  int buttonIn = digitalRead(buttonPin); // Button to simulate ultrasonic sensor input

  if(buttonIn == High) {
    initialiseDetector(); // Feels like it should be its own class. Will work on that
  }
  detectColour(sensorVal);  
}

void initialiseDetector() {
  if(state == 0) { // Ensure no interruptions
    state = 1;
  }
}

void detectColour(int sensorVal) {
  currMillis = millis();

  if(currMillis - prevMillis >= interval) {
    switch(state) {
      case 0: // State 0: Dormant
        break;
      case 1: // State 1: Initialised
        pastSamples.resetFill(1023);
        state = 2;
        counter = 0;
        isBlue = 0;
        isRed = 0;
        Serial.println("Recording");
        break;
      case 2: // State 2: Recording
        int prevVal = pastSamples.pop();

        Serial.print(sensorVal);
        Serial.print(" ");
        Serial.println(prevVal);

        pastSamples.add(sensorVal);

        if(sensorVal - prevVal >= blueThreshold) {
          Serial.println("Blue!");
          isBlue = 1;
          isRed = 0;
          state = 0;
          counter = 0; // Reset counter (just in case)
        } else {
          counter++;
          if(counter >= counterThresh) {
            Serial.println("Red!");
            isBlue = 0;
            isRed = 1;
            state = 0;
            counter = 0; // Reset counter (just in case)
          }
        }
        break;
      default:
        Serial.println("Illegal state encountered.");
        break;
    prevMillis = currMillis;
  }
}
