unsigned long startTime;          // Variable to store the start time
const unsigned long duration = 60000;  // Duration to run (1 minute = 60000 ms)

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  startTime = millis();           // Record the start time
  Serial.println("Time(ms),SoundLevel"); // CSV headers for timestamp and sound level
}

// the loop routine runs over and over again forever:
void loop() {
  if (millis() - startTime >= duration) {
    Serial.println("1 minute has passed. Stopping the recording.");
    while (true) {
      // Infinite loop to stop further execution
    }
  }
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  unsigned long currentTime = millis();
  
  // print out the value you read:
  Serial.print(currentTime);
  Serial.print(",");
  Serial.println(sensorValue);
  delay(0.1);
} 
