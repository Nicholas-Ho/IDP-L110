// Ultrasonic sensor (basic distance return to serial port)

int trigPin = 1;
int echoPin = 0;

long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Clear the trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Set a HIGH pulse on trigger for 10 us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read echoPin time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculate distance (cm)
  distance = duration * 0.034/2;
  // Print to Serial Monitor, distance in cm
  Serial.print("\nDistance: \n");
  Serial.print(distance);
}