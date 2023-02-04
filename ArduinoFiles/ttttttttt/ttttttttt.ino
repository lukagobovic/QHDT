// Serial Write
int counter = 0;
char telemetry[33];
void setup() {
  // put your setup code here, to run once:
  //Serial3.begin(9600);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    int bytes = Serial.readBytesUntil((char)0x4, telemetry, 4);
    Serial.print("X:");
    Serial.println((byte)telemetry[0]);
    Serial.print("Y:");
    Serial.println((byte)telemetry[1]);
    Serial.print("Z:");
    Serial.println((byte)telemetry[2]);
  }
}
