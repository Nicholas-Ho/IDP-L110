#include "circular_buffer.h"

// Analogue pin
int colourPinIn = A0;
int isBlue = 0;

int blueThreshold = 40; // in bits
int redThreshold = 20; // in bits

// State 0: Paused
// State 1: Detecting
int state = 1;

// Checking against sample approximately 1 s ago
CircularBuffer pastSamples = CircularBuffer(100, true, 1023);

// Sample approximately every 10 ms
int interval = 10;
long prevMillis = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorVal = analogRead(colourPinIn);

  long currMillis = millis();

  if(currMillis - prevMillis >= interval) {
    int prevVal = pastSamples.pop();

    Serial.print(sensorVal);
    Serial.print(" ");
    Serial.println(prevVal);

    pastSamples.add(sensorVal);
    if(sensorVal - prevVal >= blueThreshold) {
      Serial.println("Blue!");
      state = 0;
    }
    prevMillis = currMillis;
  }
}
