#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DueCANLayer.h"
#include "Adafruit_FRAM_I2C.h"
#include "max6675.h"
#include "constants.h"
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>


enum PodState {
  INIT,
  FAULT,
  SAFE_TO_APPROACH,
  READY_TO_LAUNCH,
  LAUNCH,
  COAST,
  BRAKE,
  CRAWL,
  GROUNDWARNING,
  STDBY
};

PodState the_STATE = FAULT;

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
byte canData[60];

int speedOfMotor;
int isoState;

//LED Pins
int REDLED = 23;
int YELLOWLED = 22;
int GREENLED = 40;
int buttonPin = 45;

int ledState = LOW;
unsigned long previousMillis = 0;

//Buffers and IDs for sending and recieving stuff
char tempbuff[10] = "0";
byte dataList[60] = {};
int BMSBuffID = 5;
int ESCBuffID = 10;
int thermocoupleBuffID = 15;
int IMUBuffID = 20;
// int thermocoupleBuffID = 25;

uint32_t theID = 0;

Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
uint16_t addressIndex = 0;

File dataFile;

//GPS
TinyGPSPlus gps;
float speedKph = 0;
double START_LAT = 0.0;
double START_LON = 0.0;
double lat = 0.0;
double lon = 0.0;
double distance = 0;
bool firstTime = true;

//Thermocouple Pins
//Shared SPI
int thermoDO = 74;
int thermoCLK = 72;
//Individual CS pins
int thermoCS1 = 5;
int thermoCS2 = 6;
int thermoCS3 = 7;

//Thermocouple declarations
MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);

/**************************************************************************/
//CAN BMS Structs
struct data356 {
  int16_t voltage;
  int16_t current;
  int16_t temp;
};

struct data356 ID356(CAN_FRAME frame) {
  //V, A, temp
  data356 data;
  data.voltage = frame.data.s0;
  data.current = frame.data.s1;
  data.temp = frame.data.s2;
  return data;
}

struct data373 {
  uint16_t minCellVoltage;
  uint16_t maxCellVoltage;
  uint16_t lowestCellTemp;
  uint16_t highestCellTemp;
};

struct data373 ID373(CAN_FRAME frame) {
  //  min cell V, max cell V, lowest cell temp, high cell temp
  data373 data;
  data.minCellVoltage = frame.data.s0;
  data.maxCellVoltage = frame.data.s1;
  data.lowestCellTemp = frame.data.s2;
  data.highestCellTemp = frame.data.s3;
  return data;
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

byte* createDataBuff(int buffID, byte* buffer) {
  int arrayLength = sizeof(buffer) / sizeof(buffer[0]);
  byte myArray[arrayLength];
  for (int i = 0; i < arrayLength; i++) {
    myArray[i + 1] = buffer[i];
  }
  myArray[0] = buffID;
  return myArray;
}


int readIsolationState(CAN_FRAME frameToSend) {
  Can0.sendFrame(frameToSend);
  CAN_FRAME response;
  if (Can0.read(response)) {
    if (response.id == 0xA100100) {
      byte statusBits = response.data.bytes[1];
      byte isolationState = statusBits & 0x03;
      if (isolationState == 0b10) {
        //Re-call the function and keep checking the fault, if it gets worse, then do the error stuff
        return 7;  //Special value that will set yellow led to be solid, possible warning
      } else if (isolationState == 0b11) {
        //Set the state LED to RED, there is a fault, turn the motors off right away and apply brakes
        return 0;  //FAULT STATE, stay away from pod, ground fault detected
      } else {
        return 8;  //Nothing out of the ordinary, proceed as normal
      }
    }
  }
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double earthRadius = 6371000; // in meters

  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = earthRadius * c;

  return distance;
}


bool checkSerial() {}


/**************************************************************************/
void setup(void) {
  Serial.begin(115200);
  Serial2.begin(9600); // GPS Serial Port
  Serial3.begin(9600);
  gpsSerial.begin(9600);

  while (!Serial) delay(10);  // wait for serial port to open!

  //Init SD card with CS pin at 4
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  dataFile = SD.open("test.txt", FILE_WRITE);
  if (!myFile) {
    Serial.println("error opening test.txt");
  }

  pinMode(stopLED, OUTPUT);
  pinMode(crawlLED, OUTPUT);
  pinMode(speedLED, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(stopLED, LOW);
  digitalWrite(crawlLED, LOW);
  digitalWrite(speedLED, LOW);

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  if (canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0 Initialized Successfully.\n\r");
  else
    Serial.print("CAN0 Initialization Failed.\n\r");

  delay(1000);

  bno.setExtCrystalUse(true);

  CAN_FRAME IMD_Frame;
  IMD_Frame.id = 0xA100101;
  IMD_Frame.data.bytes[0] = 0xE0;  // Multiplexor for isolation state request
  IMD_Frame.length = 1;
  IMD_Frame.extended = 1;

  //First check if there are any ground faults before proceeding with anything. Done on powerup.
  PodState currentState = STDBY;
  isoState = readIsolationState(IMD_Frame);  //Checks for ground faults
  if (isoState == 0)
    the_STATE = FAULT;
    dataFile.println("Isolation State: Good");
  else if (isoState == 7)
    the_STATE = GROUNDWARNING;
    dataFile.println("Isolation State: Warning");
  else
    the_STATE = STDBY;
    dataFile.println("Isolation State: DANGER");

  bool SerialReady = false;
  //This is where we initialize, wait until the pod is mounted and everthing is ready
  while (!SerialReady) {
    if (Serial3.available()) {
      int dataIn = Serial3.read() - '0';  // Subtract the ASCII value of '0' to get the actual value
      Serial.print("Received state: ");
      Serial.println(dataIn);
      //If data is recieved from network arduino indicating initialization state, continue with operation of pod
      if (dataIn == INIT) {
        SerialReady = true;
        dataFile.println("POD Initialized");
      }
    }
  }

  //Add a button somewhere that triggers the safe to approach state, once the pod is mounted
  bool PodMounted = false;
  while (!PodMounted) {
    int buttonValue = digitalRead(buttonPin);
    if (buttonValue == LOW) {
      // If button pushed, pod is safe to approach
      the_STATE = SAFE_TO_APPROACH;
      dataFile.println("POD Safe To Approach");
      //Delay just in case;
      delay(5000);
      PodMounted = true;
    }
  }

  //Wait until we recieve input from remote GUI to indicate pod is ready to launch
  bool RTL = false;
  while (!RTL) {
    if (Serial3.available()) {
      // Read the incoming data and convert it to an integer
      int theState = Serial3.read() - '0';  // Subtract the ASCII value of '0' to get the actual value
      Serial.print("Received state: ");
      Serial.println(theState);
      if (theState = READY_TO_LAUNCH) {
        RTL = true;
        dataFile.println("POD Ready To Launch");
      }
    }
  }

  bool groundFault = false;
  bool lossOfSerial = false;
  //Do not proceed with procedures unless we are sure there is no ground faults and that we have serial connnection
  //while (!groundFault && !lossOfSerial) {
  bool launch = false;
  while (!launch) {
    //Perform the ready to lauch procedures (brakes still on, blinking yellow lights, apply current brake)
    //Always keep checking for ground faults and for loss of comms on radios
    isoState = readIsolationState(IMD_Frame);  //Checks for ground faults
    if (isoState == 0)
      faultSTATE();
    if (Serial3.available()) {
      int theState = Serial3.read() - '0'; 
      if (theState = -1) {
        faultSTATE();
      }
    }
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 750) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(YELLOWLED, ledState);
    }
    if (Serial3.available()) {
      // Read the incoming data and convert it to an integer
      int theState = Serial3.read() - '0';  // Subtract the ASCII value of '0' to get the actual value
      Serial.print("Received state: ");
      Serial.println(theState);
      if (theState = LAUNCH) {
        //Recieved state is launch, delay 10 seconds, allowing everyone to clear the pod again
        delay(10000);
        launch = true;
      }
    }
  }

  //We are now in the launch phase
  //Brakes off, motors on full beans
  unsigned long timeSinceLaunch = millis();  //if this exceeds certain time, apply brakes
  bool inLaunch = false;
  bool coast = false;
  bool brake = false;

  while (!inLaunch) {
    isoState = readIsolationState(IMD_Frame);  //Checks for ground faults
    if (isoState == 0)
      faultSTATE();
      dataFile.println("Isolation State: DANGER");
    if (Serial3.available()) {
      int theState = Serial3.read() - '0'; 
      if (theState = -1) {
        faultSTATE();
        dataFile.println("No Connection: FAULT");
      }
    }
    digitalWrite(YELLOWLED, LOW);
    digitalWrite(GREENLED, HIGH);

    //Just one motor for now to test on motor
    canTx(0, 0x00000349, true, highSpeed, 8);

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid() && gps.speed.isValid()) {
      speedKph = gps.speed.kmph();
      dataFile.print("Speed: ");
      dataFile.println(speedKph);


    //Watch for distance and time since launch, and watch for speed
    //If speed is too high, the coast
    //Coast until reached max distance or  time since launch experied
    if (gps.location.isValid())
    {
      if(firstTime){
        START_LAT = gps.location.lat();
        START_LON = gps.location.lng();
        firstTime = false;
      }
      distance = calculateDistance(
        START_LAT, START_LON,
        gps.location.lat(), gps.location.lng()
      );
      Serial.println(distance);
      dataFile.print("Distance: ");
      dataFile.println(distance);
    }
    if (speedKph >= LAUNCH_SPEED_THRESH) {
      inLaunch = true;
      the_STATE = COAST;
    } else if (distance >= BRAKE_DIST_THRESH || timeSinceLaunch - millis() >= LAUNCH_TIME_THRESH) {
      inLaunch = true;
      the_STATE = BRAKE;
    }
  }

  bool nextState = false;
  //Now in the coasting state, only can go into the brake state once the distance exceeded or the time since launch exceeds the coast thresh
  while (!nextState) {
    isoState = readIsolationState(IMD_Frame);  //Checks for ground faults
    if (isoState == 0)
      faultSTATE();
    if (Serial3.available()) {
      int theState = Serial3.read() - '0'; 
      if (theState = -1) {
        faultSTATE();
      }
    }
    while (Serial2.available() > 0)
      gps.encode(Serial3.read());

    if (gps.location.isValid() && gps.speed.isValid()) {
      speedKph = gps.speed.kmph();
      dataFile.print("Speed: ");
      dataFile.println(speedKph);
    }

    if (gps.location.isValid())
    {
        distance = calculateDistance(
        START_LAT, START_LON,
        gps.location.lat(), gps.location.lng()
      );
      Serial.println(distance);
      dataFile.print("Distance: ");
      dataFile.println(distance);
    }
    if (the_STATE = COAST) {
      canTx(0, 0x00000349, true, crawl, 8);
      if (distance >= BRAKE_DIST_THRESH || timeSinceLaunch - millis() >= LAUNCH_TIME_THRESH) {
        the_STATE = BRAKE;
      }
    } else if (the_STATE = BRAKE) {
      //Set current brake on motors to however many amps(figure out)
      canTx(0, 0x00000349, true, current, 8);
      applyBrakes();
    }
  }
}
}


void processState(PodState state) {
  switch ()
}

void loop(void) {



  //Now the pod is mouted on the track, we need to apply pressure to the brakes.
  //LEARN HOW TO APPLY BRAKES





















  int dataListIndex = 1;
  CAN_FRAME incoming;
  Can0.read(incoming);
  for (int i = 0; i < incoming.length; i++) {
    canData[i] = incoming.data.bytes[i];
  }
  canData = createDataBuff(canBuffID, canData);

  Serial3.write(canData, 20);  //Sends CAN data to
  delay(500);

  if (Serial3.available()) {
    // Read the incoming data and convert it to an integer
    int theState = Serial3.read() - '0';  // Subtract the ASCII value of '0' to get the actual value
    Serial.print("Received state: ");
    Serial.println(theState);
    speedOfMotor = theState;
    speedSelect(theState);
  }
  //Sets the speed of the motor
  canTx(0, 0x00000349, true, tempdata, 8);

  /* Display the floating point data */
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
