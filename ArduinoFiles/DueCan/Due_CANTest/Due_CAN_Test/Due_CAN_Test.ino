#include "DueCANLayer.h"

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);


int speedOfMotor;
int realSpeed;
boolean didItAlready = false;
char cmd[2];
volatile int v2 = 0;
char tempbuff[10] = "0";

uint32_t theID = 0;

void setup()
{
  // Set the serial interface baud rate
  Serial.begin(115200);
  Serial3.begin(9600);

  // Initialize CAN controller
  if (canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0 Initialized Successfully.\n\r");
  else
    Serial.print("CAN0 Initialization Failed.\n\r");



}// end setup

void printFrame(CAN_FRAME &frame) {
   
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


void loop()
{
  // Declarations
  //This array is simply used to speed up and slow down the motor every few seconds, kinda pointless
  byte cTxData[7][8] = {{0x00, 0x00, 0x09, 0xc4, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1d, 0x4c, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x27, 0x10, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1d, 0x4c, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x09, 0xc4, 0x00, 0x00, 0x00, 0x00}
  };

  //Temp data array to copy to when we want to change data
  byte tempdata[] = {0x00, 0x00, 0x09, 0xc4, 0x00, 0x00, 0x00, 0x00};

  //Three various speeds that can be used
  /*
   * The data segment for CAN is 8 bytes, so each index in the array is one byte
   * for the VESC we are using, the first 4 bytes, specify the motor speed. 
   * KEEP IN MIND THESE ARE HEX VALUES, SO DO NOT PUT 5000 RPM thinking that will give you
   * 5000 rpm on the motor!! the lowest value is the 4th element in this array, then begin 
   * working your way up. The crawl speed in this case is 0x0000009c4, which is 2500 RPM.
   * 
   * The next two byte specify the motor current, in amps, and the last 2 specify the duty cycle
   * We will not really be focusing on these as once we set the RPM, the motor controller will
   * set the other two respecitvely
   * 
   * When monitoring, all three values will be present, which is how we can read the data from the controller. 
   * The main things we will be focusing on are the RPM and current
   * 
   * Keep in mind the RPM here is specified in ERPM, which is not true RPM. If a motor has 7 windings, the true
   * RPM is ERPM/7, so in the crawl speed that would be 2500/7. Our actual motors have 5 windings, so keep that 
   * in mind
   * 
   * Read more into the document That I sent you guys for how to ID system works in CAN, but essentially,
   * there is an id for what the command is to do/what data is coming in, and an id for sending a command
   */
  byte fullstop[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte crawl[] = {0x00, 0x00, 0x09, 0xc4, 0x00, 0x00, 0x00, 0x00};
  byte mediumSpeed[] = {0x00, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00};
  byte highSpeed[] = {0x00, 0x00, 0x48, 0x5c, 0x00, 0x00, 0x00, 0x00};

  int nTimer = 0;
  int counter = 0;

  while (1) // Endless loop
  {
//    if (counter == 7) {
//      counter = 0;
//    }
//    Serial.println(counter);
    delay(10);
    CAN_FRAME incoming;
    Can0.read(incoming);
    theID = incoming.id;
//     Serial.println(theID);
    Serial3.write(theID); 

    //printFrame(incoming);
//    for (int i = 0; i < 8; i++) {
//      tempdata[i] = cTxData[counter][i];
//    }

    // Send the message every 1000 milliseconds
      if(Serial3.available()){
        speedOfMotor = Serial3.read();
        Serial.println(speedOfMotor);
        switch (speedOfMotor) {
          case 48 :
            Serial.println("slow");
            memcpy(tempdata, fullstop, 8);
            break;
          case 49 : 
            Serial.println("slow");
            memcpy(tempdata, crawl, 8);
            break;
          case 50:
            Serial.println("medium");
            memcpy(tempdata, mediumSpeed, 8);
            break;
          case 51:
            Serial.println("high");
            memcpy(tempdata, highSpeed, 8);
            break;
        }
      }
      
      canTx(0, 0x0000036f, true, tempdata, 8);
////    if (++nTimer == 1000)
////    {
//      if (canTx(0, 0x00000339, true, tempdata, 8) == CAN_OK)
//        Serial.println("");
//      else
//        Serial.println("");
//
//      nTimer = 0;
//      counter++;
////    }// end if
      


    // Check for received message
    //    long lMsgID;
    //    bool bExtendedFormat;
    //    byte cRxData[8];
    //    byte cDataLen;
    //    if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
    //    {
    //      Serial.print("Rx - MsgID:");
    //      Serial.print(lMsgID, HEX);
    //      Serial.print(" Ext:");
    //      Serial.print(bExtendedFormat);
    //      Serial.print(" Len:");
    //      Serial.print(cDataLen);
    //      Serial.print(" Data:");
    //
    //      for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
    //      {
    //        Serial.print(cRxData[cIndex], HEX);
    //        Serial.print(" ");
    //      }// end for
    //
    //      Serial.print("\n\r");
    //
  }// end if

}// end while

// end loop
