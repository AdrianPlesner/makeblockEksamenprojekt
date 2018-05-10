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
  manualDrive,
  automatic,
};
STATES state;

enum AUTOSTATES
{
  Begin,
  drive,
  avoidObject,
  turnCorner,
  correctError,
  done
};
AUTOSTATES autostate;

unsigned char table[16] = {0}; 
//int DSpeed = 0;

int USoffset = 5; //Ultrasonic distance offset is 5 cm to robot border

float encoderValue[100][4];

int encoderIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  state = idle;
  Serial3.write(0);
  ctrlMotor.Begin();
  TCCR3A = 0;
  TCCR3B = 0;
  // initial counter value = 0
  TCNT3 = 0;
  //Setting compare match register to 100 Hz
  OCR3A = 19999;
  // turn on CTC (compare mathc) mode
  TCCR3A |= (1 << WGM32);
  // Set CS31 bit for 8 bit prescaler
  TCCR3B |= (1 << CS31);
  //enable timer interrupt
  TIMSK3 |= (1 << OCIE3A);
  

}

void loop() {

switch(state)
  {
    case idle:
      ComReadData();
      table[0] = 0; //state
      table[1] = 0; //x speed
      table[2] = 0; //y speed
      table[3] = 0; // rotational speed
      Serial.println("read data");  
      Serial.print(table[0] );
      Serial.print(" " );
      Serial.print(table[1]);
      Serial.print(" " );
      Serial.println(table[2]);
      state = table[0];  
      ctrlUS.getDist();  
      Serial.print("USl ");
      Serial.println(ctrlUS.lDist);
      Serial.print("USf ");      
      Serial.println(ctrlUS.fDist);
      
      break;
      
    case manualDrive:
      /*Serial.println("in drive");
      Serial.println(table[0]);
      Serial.println(table[1]);
      Serial.println(table[2]);*/

      ctrlMotor.drive(table[1],table[2],table[3]);

      //state = idle;
      break;

    case automatic:
      //Collision scanner
      switch (autostate)
      {
        case Begin:
          
        break;
        case drive:

        break;
        case avoidObject:

        break;
      }
      

      
      
    break;
  }
}


// REMINDER! Certain parts of the libraries/makeblock/src/utility/avr/servo.cpp library was removed in order to use this
ISR(TIMER3_COMPA_vect) //short function for what happens at the interrupt
{
  Serial.print(ctrlMotor.getVelocity(1));
  Serial.print(" ");
  Serial.print(ctrlMotor.getVelocity(2));
  Serial.print(" ");
  Serial.print(ctrlMotor.getVelocity(3));
  Serial.print(" ");
  Serial.println(ctrlMotor.getVelocity(4));
  /*for(int i = 0; i<4;i++)
  {
    encoderValue[i][encoderIndex] = ctrlMotor.getVelocity(i+1);
    Serial.print(ctrlMotor.getVelocity(i+1));
  }
  Serial.println();
  encoderIndex++;*/
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


