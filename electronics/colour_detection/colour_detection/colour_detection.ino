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
  digitalWrite(bluePinOut, HIGH);
  digitalWrite(redPinOut, HIGH);
  delay(1000);
  digitalWrite(bluePinOut, HIGH);
  digitalWrite(redPinOut, LOW);
  Serial.begin(9600);
}

void loop() {
  int sensorVal = analogRead(colourPinIn);
  int buttonIn = digitalRead(buttonPin); // Button to simulate ultrasonic sensor input

  detector.initialiseDetector();
  
  int colour = detector.detectColour(sensorVal);
  isBlue = colour == 1;
  isRed = colour == 2;

  if(isBlue == true) {
    Serial.println("Blue!!!");
    digitalWrite(bluePinOut, HIGH);
    digitalWrite(redPinOut, LOW);
    delay(1000);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
  } else if(isRed == true) {
    Serial.println("Red!!!");
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, HIGH);
    delay(1000);
    digitalWrite(bluePinOut, LOW);
    digitalWrite(redPinOut, LOW);
  }
}