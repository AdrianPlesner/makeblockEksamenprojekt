#include <Arduino.h>
#include "Classes.h"
#include <Wire.h>
#include <SoftwareSerial.h>

motor ctrlMotor;
USSens ctrlUS;
gyroSens ctrlGyro;

enum STATES
{
  idle,
  drive,
  automatic,
};
STATES state;

unsigned char table[16] = {0}; 
int DSpeed = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  state = idle;
  Serial3.write(0);
  

}

void loop() {

switch(state)
  {
    case idle:
      ComReadData();
      /*table[0] = 1;
      table[1] = 0;
      table[2] = 0;
      table[3] = 0;*/
      Serial.println("read data");  
      Serial.print(table[0] );
      Serial.print(" " );
      Serial.print(table[1]);
      Serial.print(" " );
      Serial.println(table[2]);
      state = table[0];    
      Serial.print("Ultra Sonic Left ");
      Serial.println(ctrlUS.lDist);
      Serial.print("Ultra Sonic Front ");      
      Serial.println(ctrlUS.fDist);
      
      break;
      
    case drive:
      Serial.println("in drive");
      Serial.println(table[0]);
      Serial.println(table[1]);
      Serial.println(table[2]);

      //Drive
      //DSpeed = table[2];
      DSpeed = 220;
      driveing();
      state = idle;
      break;

    case automatic:
      //Collision scanner
      
      

      
      
    break;
  }
}



void driveing()
{
  
  
     
}







void ComReadData()
{
  int readdata = 0,i = 0,count = 0;
  char outDat;
  if (Serial3.available())
  {
    while((readdata = Serial3.read()) != (int)-1)
    {
      table[count] = readdata;
      count++;
      delay(1);
    }
  }
}


