void setup() {
  Serial.begin(9600); //initialize serial communication
}

void loop() {
  if (Serial.available() > 0) { //check if there is data available
    int receivedInt = Serial.parseInt(); //read the integer sent via serial

    //do something with the received integer, such as print it to the serial monitor
    Serial.println(receivedInt);
  }
}
