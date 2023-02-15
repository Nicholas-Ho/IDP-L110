int sensorPin = A5;    // select the input pin for the potentiometer

int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(9600);          //Start serial and set the correct Baud Rate        
}

void loop() {
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
}