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

//arrays of sensor values
int encoderValue[100][4];
int gyroValue[100];

int encoderIndex = 0;

// d+l = 19
// Constant multiplication array for calculation wheel velocities
int dirKin[3][4] = { {1, 1, 1, 1},
                   {-1, 1, -1, 1},
                   {-1/19, -1/19, 1/19, 1/19}
                 };

// Array of wanted velocities, xdot, ydot, phidot
int aryEncoder[4][1] = { {0},
                   {0},
                   {0},
                   {0}
                 };
// Array of resulting wheel velocities
int aryWheel[3][1];

int lastPos[3];

int currentPos[3];

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  state = idle;
  Serial3.write(0);
  ctrlMotor.Begin();

  noInterrupts(); //There should be no interrupts while the timer interrupt is initialized
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
  interrupts(); //Interrupts can happen again now
  

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
        
      Serial.print("USl ");
      Serial.println(ctrlUS.getDist(false));
      Serial.print("USf ");      
      Serial.println(ctrlUS.getDist(true));
      
      break;
      
    case manualDrive: // Manual drive directly drives from indputvalues Xm, Ym and PHIm
      /*Serial.println("in drive");
      Serial.println(table[0]);
      Serial.println(table[1]);
      Serial.println(table[2]);*/

      ctrlMotor.drive(table[1],table[2],table[3]);

      //state = idle;
      break;

    case automatic:
      //Collision scanner
      estimatePos();
      switch (autostate)
      {
        case Begin:
        lastPos[0] = 0; lastPos[1] = 0; lastPos[2] = 0;
          if(ctrlUS.getDist(true) > 10 && ctrlUS.getDist(false) > 10) //long distance to both walls
          {
            
          }
          else if(ctrlUS.getDist(true) > 10 && ctrlUS.getDist(false) <= 10) //only front distance is long
          {
            autostate = drive;
          }
          else if(ctrlUS.getDist(true) <= 10 && ctrlUS.getDist(false) > 10) //only left distance is long
          {
            
          }
          else //both left and front distances are short
          {
            autostate = turnCorner;
          }
        break;
        case drive:

        break;
        case avoidObject:

        break;
        case turnCorner:

        break;
        case correctError:

        break;
        case done:
          state = idle;

        break;
      }
      

      
      
    break;
  }
}


// REMINDER! Certain parts of the libraries/makeblock/src/utility/avr/servo.cpp library was removed in order to use this
ISR(TIMER3_COMPA_vect) //short function for what happens at the interrupt
{
  /*Serial.print(ctrlMotor.getVelocity(1));
  Serial.print(" ");
  Serial.print(ctrlMotor.getVelocity(2));
  Serial.print(" ");
  Serial.print(ctrlMotor.getVelocity(3));
  Serial.print(" ");
  Serial.println(ctrlMotor.getVelocity(4));*/
  for(int i = 0; i<4;i++)
  {
    encoderValue[i][encoderIndex] = ctrlMotor.getVelocity(i+1);
    gyroValue[encoderIndex] = ctrlGyro.getRotation();
    //Serial.print(ctrlMotor.getVelocity(i+1));
  }
  //Serial.println();
  encoderIndex++;
}
float rToM = 0.523; //constant for calculating cm/s from rpm

void estimatePos() //Function to calculate the currentposition of the robot
{
  int motorVel[100][4];
  int roboVel[100][3];
  int resPos[3];
  noInterrupts(); //there shoud be no interrupts as the encoder values are read
  for(int i = 0; i < 100; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      motorVel[i][j] = encoderValue[i][j] * rToM;
    }
  }
  encoderIndex = 0;
  interrupts();
  for(int i = 0; i < 100; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      aryEncoder[j][0] = motorVel[i][j];
    }
    calcWheelVel(dirKin, aryEncoder, aryWheel); //Calculating robot velocities from encoder velocities via direct kinematic model
    for(int j = 0; j < 3; j++)
    {
      roboVel[i][j] = aryWheel[j][0];
    }
  }
  for(int i = 0; i < 100; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      resPos[j] = resPos[j] + roboVel[i][j]*0.01; //Calculating the difference in position from the sum of the velosities times the time interval
    }
  }
  for(int i = 0; i < 3; i++)
  {
    currentPos[i] = currentPos[i] + resPos[i]; 
  }
}


// Function for calculating wheel velocities
void calcWheelVel(int matA[3][4], int matB[4][1], int resMat[3][1])
{
  double a = 0;
  for(int i = 0; i < 3;i++)
  {
    for(int j = 0; j < 1; j++)
    {
      int a = 0;
      for(int k = 0; k < 4; k++)
      {
        a += matA[i][k] * matB[k][j];
      }
      resMat[i][j] = a;
    }
  }  
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


