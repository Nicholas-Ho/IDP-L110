#include "colour_detector.h"

// Analogue pin
int colourPinIn = A0;
int bluePinOut = 7;
int redPinOut = 8;
int isBlue = 0;
int isRed = 0;

// Digital pin (for test button)
int buttonPin = 2;

ColourDetector detector = ColourDetector();

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(bluePinOut, OUTPUT);
  pinMode(redPinOut, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorVal = analogRead(colourPinIn);
  int buttonIn = digitalRead(buttonPin); // Button to simulate ultrasonic sensor input

  if(buttonIn == HIGH) {
    detector.initialiseDetector();
  }
  
  int colour = detector.detectColour(sensorVal);
  isBlue = colour == 1;
  isRed = colour == 2;

  if(isBlue == true) {
    digitalWrite(bluePinOut, HIGH);
    digitalWrite(redPinOut, LOW);
    delay(500);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
  } else if(isRed == true) {
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, HIGH);
    delay(500);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
  }
}