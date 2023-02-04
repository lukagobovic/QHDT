#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_MLX90614.h>

const int AirValue = 474 ;   //you need to replace this value with Value_1
const int WaterValue = 209;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent = 0;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

RF24 radio(9,10); // CE, CSN
const byte address[6] = "00001";
void setup() {
  mlx.begin();
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  const char text[] = "Hello World";
  float num[32];
  num[0] = mlx.readAmbientTempC();
  soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
  Serial.println(soilMoistureValue);
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  num[1] = soilmoisturepercent;
  radio.write(&num, sizeof(num));
  delay(1000);
}
