// Ultrasonic sensor (basic distance return to serial port)
// Uses New Ping library (https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home)
#include <NewPing.h>

int trigPin = 3;
int echoPin = 2;
int maxDist = 10; // In cm

NewPing sonar(trigPin, echoPin, maxDist);

void setup() {
  Serial.begin(9600);
  Serial.println("Initialised.");
  Serial.print("Maximum Distance: ");
  Serial.println(maxDist);
}

void loop() {
  delay(1000);
  detectUltrasonic();
}

void detectUltrasonic() {
  int dist = sonar.ping_cm();
  if(dist != 0) {
    Serial.print("Ping: ");
    Serial.print(dist);
    Serial.println("cm");
  } else {
    Serial.println("No object detected.");
  }
}
