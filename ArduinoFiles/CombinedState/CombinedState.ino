#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DueCANLayer.h"
#include "Adafruit_FRAM_I2C.h"

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

// CAN VARIABLES and CONSTANTS
byte fullstop[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte crawl[] = { 0x00, 0x00, 0x09, 0xc4, 0x00, 0x00, 0x00, 0x00 };
byte mediumSpeed[] = { 0x00, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00 };
byte highSpeed[] = { 0x00, 0x00, 0x48, 0x5c, 0x00, 0x00, 0x00, 0x00 };
byte tempdata[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte current[] = { 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00 };


int speedOfMotor;
int stopLED = 23;
int crawlLED = 22;
int speedLED = 40;
int realSpeed;
boolean didItAlready = false;
char cmd[2];
volatile int v2 = 0;
char tempbuff[10] = "0";
byte canData[60];

uint32_t theID = 0;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;  //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500;             // how often to print the data
uint16_t printCount = 0;                   //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251;  //trig functions require radians, BNO055 outputs degrees
int dataID = 15;

byte dataList[60] = {};

Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
uint16_t addressIndex = 0;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);
  Serial3.begin(9600);

  while (!Serial) delay(10);  // wait for serial port to open!

  fram.begin(0x50);

  pinMode(stopLED, OUTPUT);
  pinMode(crawlLED, OUTPUT);
  pinMode(speedLED, OUTPUT);
  digitalWrite(stopLED, LOW);
  digitalWrite(crawlLED, LOW);
  digitalWrite(speedLED, LOW);




  /* Initialise the sensor */
  // if (!bno.begin()) {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1)
  //     ;
  // }

  if (canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0 Initialized Successfully.\n\r");
  else
    Serial.print("CAN0 Initialization Failed.\n\r");

  delay(1000);

  bno.setExtCrystalUse(true);
}

void printFrame(CAN_FRAME& frame) {

  Serial.print("ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data: 0x");
  for (int count = 0; count < frame.length; count++) {
    Serial.print(frame.data.bytes[count], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\n");
}

void speedSelect(int speed) {
  switch (speed) {
    case 0:
      Serial.println("off");
      digitalWrite(stopLED, HIGH);
      digitalWrite(crawlLED, LOW);
      digitalWrite(speedLED, LOW);
      memcpy(tempdata, fullstop, 8);
      break;
    case 1:
      Serial.println("slow");
      digitalWrite(stopLED, LOW);
      digitalWrite(crawlLED, HIGH);
      digitalWrite(speedLED, LOW);
      memcpy(tempdata, crawl, 8);
      break;
    case 2:
      Serial.println("medium");
      memcpy(tempdata, mediumSpeed, 8);
      digitalWrite(stopLED, LOW);
      digitalWrite(crawlLED, LOW);
      digitalWrite(speedLED, HIGH);
      break;
    case 3:
      Serial.println("high");
      memcpy(tempdata, highSpeed, 8);
      digitalWrite(stopLED, LOW);
      digitalWrite(crawlLED, LOW);
      digitalWrite(speedLED, HIGH);
      break;
  }
}

void loop(void) {
  int dataListIndex = 1;
  CAN_FRAME incoming;
  Can0.read(incoming);
  canData[0] = 14;
  for (int i = 0; i < incoming.length; i++) {
    canData[i + 1] = incoming.data.bytes[i];
  }
  // Serial.println(theID);
  Serial3.write(canData, 20);
  delay(500);
  //printFrame(incoming);

  if (Serial3.available()) {
    // Read the incoming data and convert it to an integer
    int theState = Serial3.read() - '0';  // Subtract the ASCII value of '0' to get the actual value
    Serial.print("Received state: ");
    Serial.println(theState);
    speedOfMotor = theState;
    speedSelect(theState);
  }
  canTx(0, 0x00000349, true, tempdata, 8);

  /* Display the floating point data */

  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  //enough iterations have passed that we can print the latest data
  // Serial.print("Heading: ");
  // Serial.println(orientationData.orientation.x);
  // Serial.print("Position: ");
  // Serial.print(xPos);
  // Serial.print(" , ");
  // Serial.println(yPos);
  // Serial.print("Speed: ");
  // Serial.println(headingVel);
  // Serial.println("-------");

  dataList[0] = dataID;
  // dataList[dataListIndex++] = orientationData.orientation.x;
  uint8_t orientation1 = orientationData.orientation.x;
  dataList[dataListIndex++] = orientation1 >> 8;
  dataList[dataListIndex++] = orientation1 & 0xFF;
  if (addressIndex < 8192) {
    fram.write(addressIndex++, orientation1 >> 8);
    fram.write(addressIndex++, orientation1 & 0xFF);
  }
  dataList[dataListIndex++] = (int)yPos >> 8;
  dataList[dataListIndex++] = (int)yPos & 0xFF;
  dataList[dataListIndex++] = (int)headingVel;
  Serial3.write(dataList, 20);
  // Serial.print("X: ");
  // Serial.print(euler.x());
  // Serial.print(" Y: ");
  // Serial.print(euler.y());
  // Serial.print(" Z: ");
  // Serial.print(euler.z());
  // Serial.print("\t\n");


  delay(100);
}
