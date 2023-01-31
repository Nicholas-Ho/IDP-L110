// Ultrasonic sensor (basic distance return to serial port)

int trigPin = 1;
int echoPin = 0;

unsigned long prevMicros = 0;
const long intervals[2] = {2, 12}; // Note: intervals from start of cycle! So between the first and second events there are 10 seconds.

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int distance = detectUltrasonic();

  // Print to Serial Monitor, distance in cm
  Serial.print("\nDistance: \n");
  Serial.print(distance);
}

// Detect ultrasonic. Uses millis to ensure that it can run concurrently with other code and not interfere with delay.
void detectUltrasonic() {

  unsigned long currMicros = micros(); // Overflows after 70 minutes

  // Clear the trig pin
  digitalWrite(trigPin, LOW);

  if(currMicros-prevMicros >= intervals[0] && currMicros-prevMicros <= intervals[1]) { // Delay 2 microseconds to ensure trigger is cleared

    // Set a HIGH pulse on trigger for 10 us
    digitalWrite(trigPin, HIGH);

  } else if (currMicros-prevMicros >= intervals[1]) { // Delay 10 microseconds to set the trigger to LOW
    
    digitalWrite(trigPin, LOW);

    // Read echoPin time in microseconds
    long duration = pulseIn(echoPin, HIGH);
    
    // Calculate distance (cm)
    int distance = duration * 0.034/2;

    // Set prevMicros
    prevMicros = currMicros;

    return distance;

  }
}