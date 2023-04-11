#include <due_can.h>


int ID35f(){
//  battery model, firmware version, online cap in Ah
}

int ID351(){
//  charge voltage limit, max charge current, max discharge current, discharge voltage
}

struct data356{
  int16_t voltage;
  int16_t current;
  int16_t temp;
};

struct data356 ID356(CAN_FRAME frame){
  //V, A, temp
  data356 data;
  data.voltage = frame.data.s0;
  data.current = frame.data.s1;
  data.temp = frame.data.s2;
  return data;
}

struct data373
  {
    uint16_t minCellVoltage;
    uint16_t maxCellVoltage;
    uint16_t lowestCellTemp;
    uint16_t highestCellTemp;
  };

struct data373 ID373(CAN_FRAME frame){
//  min cell V, max cell V, lowest cell temp, high cell temp
  data373 data;
  data.minCellVoltage = frame.data.s0;
  data.maxCellVoltage = frame.data.s1;
  data.lowestCellTemp = frame.data.s2;
  data.highestCellTemp = frame.data.s3;
  return data;
}

void setup(){
  Can0.begin(500000, 62);
  Can0.enable();
  Serial.begin(115200);

}

void loop(){
  Can0.watchFor();
  CAN_FRAME myFrame;

  if (Can0.rx_avail()){
    //Serial.println(Can0.available());
    Can0.read(myFrame);
    //if(myFrame.id == 0x356){
      data356 data1 = ID356(myFrame);
      Serial.print("ID: ");
      Serial.println(myFrame.id, HEX);
      Serial.print("Voltage: ");
      Serial.println(data1.voltage);  
    //}else if(myFrame.id == 0x373){
      data373 data = ID373(myFrame);
      Serial.print("ID: ");
      Serial.println(myFrame.id, HEX);
      Serial.print("Min Cell Voltage: ");
      Serial.println(data.minCellVoltage);
      Serial.print("Max Cell Voltage: ");
      Serial.println(data.maxCellVoltage);
      Serial.print("Lowest Cell Temp: ");
      Serial.println(data.lowestCellTemp);
      Serial.print("Highest Cell Temp: ");
      Serial.println(data.highestCellTemp);
   // }
  }
  else {
    //Serial.println("Fucking Nope!");
  }



}
