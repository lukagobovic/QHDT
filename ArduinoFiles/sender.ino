void setup() {
  Serial3.begin(9600); //initialize serial communication
}

void loop() {
  int myIntList[] = {1, 2, 3, 4, 5}; //list of integers to send
  int listLength = sizeof(myIntList)/sizeof(myIntList[0]); //determine the length of the list

  for (int i = 0; i < listLength; i++) { //send each integer one by one
    Serial3.println(myIntList[i]); //send integer via serial
    delay(500); //wait for half a second
  }
}
