// Ultrasonic sensor (basic distance return to serial port)

int trigPin = 3;
int echoPin = 2;

unsigned long prevMicros = 0;
const long intervals[2] = {10000, 60000}; // Note: intervals from start of cycle! So between the first and second events there are 10 seconds.
int duration = 0;
unsigned long startTimeMicros = 0;

// For asynchronous reading of the pulse. Replaces the synchronous pulseIn()
// State 0: To set the trigger to LOW
// State 1: To start timing when Echo Pin is HIGH
// State 2: To stop timing when Echo Pin is LOW
int state = 0;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  detectUltrasonic();
}

// Detect ultrasonic. Uses micros to ensure that it can run concurrently with other code and not interfere with delay.
void detectUltrasonic() {

  unsigned long currMicros = micros(); // Overflows after 70 minutes

  // Serial.println(prevMicros);
  Serial.println(currMicros);

  if(state==0) {

    // Clear the trig pin
    digitalWrite(trigPin, LOW);

    // Next state
    state = 1;

    Serial.println("Initialised");

  } else if(currMicros-prevMicros >= intervals[0] && state==1) { // Delay 2 microseconds to ensure trigger is cleared

    // Set a HIGH pulse on trigger for 10 us
    digitalWrite(trigPin, HIGH);

    // Next state
    state = 2;

    Serial.println("Pulse");

  } else if (currMicros-prevMicros >= intervals[1] && state == 2) { // Delay 10 microseconds to set the trigger to LOW
  
    digitalWrite(trigPin, LOW); // End the trigPin pulse
    Serial.println("End");
    state = 0;

    attachInterrupt(echoPin, inSignal, RISING);
    attachInterrupt(echoPin, outSignal, FALLING);

    delay(1000);
  }
}

void inSignal() {
  startTimeMicros = micros();
}

void outSignal() {
  // If echo is LOW, get duration in microseconds
  long currMicros = micros();
  long duration = currMicros - startTimeMicros;

  // Calculate distance (cm)
  int distance = duration * 0.034/2;

  // Set prevMicros
  prevMicros = currMicros;

  // Print to Serial Monitor, distance in cm
  Serial.print("Distance: ");
  Serial.println(distance);
}