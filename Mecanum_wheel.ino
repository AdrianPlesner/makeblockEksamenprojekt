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
  test
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
int encoderValue[200][4];


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

float lastPos[3]; //Last confrimed position x, y, phi

float currentPos[3]; //Current calculated position

float beginPos[3]; //Position 

int runCount = 0; //Counter for number of rounds around the room, correlating distance to the walls

bool turnDir; //When turning corners true for clockwise, false for counter-clockwise

int targetPos; // The rotation the robot should have

bool testDrive = false;

void setup() {
  
  Serial.begin(115200);
  Serial3.begin(115200);
  state = idle;
  Serial3.write(0);  
  ctrlGyro.begin();
  ctrlMotor.Begin();

  noInterrupts(); //There should be no interrupts while the timer interrupt is initialized
  TCCR5A = 0;
  TCCR5B = 0;
  // initial counter value = 0
  TCNT5 = 0;
  //Setting compare match register to 100 Hz
  OCR5A = 19999;
  // turn on CTC (compare mathc) mode
  TCCR5A |= (1 << WGM52);
  // Set CS51 bit for 8 bit prescaler
  TCCR5B |= (1 << CS51);
  //enable timer interrupt
  TIMSK5 |= (1 << OCIE5A);
  interrupts(); //Interrupts can happen again now
  

}

void loop() {

switch(state)
  {
    case idle:
    {
      //noInterrupts();
      ComReadData();
      /*table[0] = 3; //state
      //for manual drive testing
      table[1] = 50; //x speed > 0 = forward, < 0 = backward
      table[2] = 0; //y speed > 0 = right, < 0 = left
      table[3] = 0; // rotational speed > 0 = counterclockwise, < 0 = clockwise
     // Serial.println("read data");  
      /*Serial.print(table[0] );
      Serial.print(" " );
      Serial.print(table[1]);
      Serial.print(" " );
      Serial.println(table[2]);*/
      state = table[0];  

      } 
    break;
      
    case manualDrive: // Manual drive directly drives from inputvalues Xm, Ym and PHIm
    {
      //Serial.println("in drive");
     // ctrlMotor.drive(table[1],table[2],table[3]);
      switch(table[1])
      {
        case 1:
        {
          ctrlMotor.mStop();
        }
        break;
        case 2:
        {
          ctrlMotor.drive(table[2],0,0);
        }
        break;
        case 3:
        {
          ctrlMotor.drive(-table[2],0,0);
        }
        break;
        case 4:
        {
          ctrlMotor.drive(0,table[2],0);
        }
        break;
        case 5:
        {
         ctrlMotor.drive(0,-table[2],0); 
        }
        break;
        case 6:
        {
          ctrlMotor.drive(table[2]*0.5,-table[2]*0.5,0);
        }
        break;
        case 7:
        {
          ctrlMotor.drive(table[2]*0.5,table[2]*0.5,0);
        }
        case 8:
        {
          ctrlMotor.drive(-table[2]*0.5, -table[2]*0.5,0);
        }
        break;
        case 9:
        {
          ctrlMotor.drive(-table[2]*0.5, table[2]*0.5, 0);
        }
        break;
        case 10:
        {
          ctrlMotor.drive(0,0,table[2]*0.1);
        }
        break;
        case 11:
        {
          ctrlMotor.drive(0,0,-table[2]*0.1);
        }
        break;
        default:
        {
          //send error message
        }
        break;
      }
      state = idle;
    }
    break;

    case automatic:
    {
      interrupts();
      delay(2000);
      //Collision scanner
      estimatePos();
      switch (autostate)
      {
        case Begin: // the first state for locating a starting point
        {
          lastPos[0] = 0; lastPos[1] = 0; lastPos[2] = 0;  // resetting initial position values
        
          if(ctrlUS.getDist(true) > 10 && ctrlUS.getDist(false) > 10) //long distance to both walls
          {
            if(ctrlUS.getDist(true) == 400 && ctrlUS.getDist(false) == 400) //No walls in reach
            {
              //Rotate to check for walls around
              int startRot = ctrlGyro.getRotation(); // the rotation of the robot when the loop starts
              bool ext = false; //should exit loop
              ctrlMotor.drive(0,0,3);
              do
              {
                if (ctrlUS.getDist(true) < 350)
                {
                  ext = true; // fund a wall
                }
                
                if ((ctrlGyro.getRotation() - startRot) < 0 && (ctrlGyro.getRotation() - startRot) > -5)
                {
                  ext = true; // no wall found
                }
                
              } while(ext == false);
              ctrlMotor.mStop();
              estimatePos();
              
            }
            //Drive to wall
            ctrlMotor.drive(0,-80,0); //drive sideways
            do
            {
              
            }while(ctrlUS.getDist(0) > 50);
            ctrlMotor.mStop();
            estimatePos();
            //rotate to find shortest distance to wall; shortest direct line
            int shortDist = ctrlUS.getDist(0);
            ctrlMotor.drive(0,0,2);
            delay(100);
            bool ext = false;
            do
            {
             if(ctrlUS.getDist(0) > shortDist)
             {
              ext = true;
             }
             else
             {
              shortDist = ctrlUS.getDist(0);
             }
             delay(50);
              
            }while(ext == false);
            
            ctrlMotor.mStop();
            estimatePos();
            
            //Drive close to wall
            ctrlMotor.drive(0,-50,0);
            do
            {
              
            }while(ctrlUS.getDist(0) > 10);
            ctrlMotor.mStop(); //At wall, start driving
            estimatePos();
            targetPos = ctrlGyro.getRotation(); // setting target rotation
            autostate = drive;
          }
          else if(ctrlUS.getDist(true) > 10 && ctrlUS.getDist(false) <= 10) //only front distance is long
          {
            autostate = drive;
          }
          else //both left and front distances are short or only left distance is long
          {
            autostate = turnCorner;
            turnDir = true;
          }
          estimatePos();
          for(int i = 0; i < 3; i++) //Setting the position when driving began
          {
            beginPos[i] = currentPos[i];
          }
          
        }
        break;
        case drive:
        {
          estimatePos();
          int i = 0;
          bool returned = false; 
          do
          {
            if(lastPos[i] > (beginPos[i] - 3) && lastPos[i] < (beginPos[i] + 3))
            {
              returned = true;
            }
            else
            {
              i++;
            }
          }while(returned == false || i < 3);
          
          if(ctrlUS.getDist(1) < wallDist() && ctrlUS.getDist(0) < wallDist() + 5 && ctrlUS.getDist(0) > wallDist() - 5) //Distance to both sides is short, turn corner clockwsie
          {
            ctrlMotor.mStop();
            autostate = turnCorner;
            turnDir = true;
          }
          else if(ctrlUS.getDist(0) > (wallDist() + 10)) // suddenly very long left distance,  turn corner counter-clockwise
          {
            ctrlMotor.mStop();
            autostate = turnCorner;
            turnDir = false;
          }
          else if(ctrlUS.getDist(0) < wallDist - 5 || ctrlUS.getDist(0) > wallDist + 5)
          {
            autostate = correctError;
          }
          else if(returned)//Completed round
          {
            ctrlMotor.mStop();
            
            ctrlMotor.drive(0,50,0);
            do // drive 5 cm in
            {
             delay(10); 
            }while(ctrlUS.getDist(0) < (wallDist + 5));
            ctrlMotor.mStop();
            estimatePos();
            for(int i = 0; i < 3; i++) //Set new begin position
            {
              beginPos[i] = lastPos[i];
            }
          }
          else // drive
          {
            ctrlMotor.drive(75,0,0);
            autostate = correctError;
          }
          
        }
        break;
        case avoidObject:
        {
          
        }
        break;
        case turnCorner:
        {
          if(turnDir == true) //clockwise
          {
            //turn 90 degrees right
            targetPos = ctrlGyro.getRotation() - 90;
            if(targetPos < 0)
            {
              targetPos = 360 + targetPos;
            }
            ctrlMotor.drive(0,0,-2);
            estimatePos();
            bool ext = false; //should exit loop
            do
            {
              if((ctrlGyro.getRotation() - targetPos) < 3 && (ctrlGyro.getRotation() - targetPos) > 0)
              {
                ext = true;
              }
            } while(ext == false);
            
          }
          else //counter-clockwise
          {
            targetPos = ctrlGyro.getRotation() + 90;
            if(targetPos > 360)
            {
              targetPos = targetPos - 360;
            }
            ctrlMotor.drive(0,0,2);
            estimatePos();
            bool ext = false; //should exit loop
            do
            {
              if((targetPos - ctrlGyro.getRotation()) < 3 && (ctrlGyro.getRotation() - targetPos) > 0)
              {
                ext = true;
              }
            } while(ext == false);
          }
          ctrlMotor.mStop();
          estimatePos();

          autostate = drive;
        }
        break;
        case correctError:
        {
          if(ctrlGyro.getRotation() > targetPos +3)
          {
            do
            {
              ctrlMotor.drive(50,0,-1); //sligtly rotate
              
            }while(ctrlGyro.getRotation() > targetPos);
          }
          else if(ctrlGyro.getRotation() < targetPos - 3)
          {
            do
            {
              ctrlMotor.drive(50,0,1); // sligtly rotate
              
            }while(ctrlGyro.getRotation() < targetPos);
          }
          autostate = drive;
        }
        break;
        case done:
        { 
          state = idle;
        }
        break;
      }
    }
    break;
    case test :
    {
      interrupts();
   // delay(100);
      //Serial.println(ctrlGyro.getRotation());
      
        ctrlMotor.drive(table[1],table[2],table[3]);
        delay(250);
        estimatePos();
       /* 
      delay(500);
      estimatePos();
        Serial.print("Sxpos: ");
        Serial.print(lastPos[0]);
        Serial.print(" Sypos: ");
        Serial.print(lastPos[1]);
        Serial.print(" Srot: ");
        Serial.println(lastPos[2]);
      ctrlMotor.drive(80,0,0);
      for(int i = 0; i < 20; i++)
      {
        delay(100);
        if(ctrlUS.getDist(1) < 15)
        {
          ctrlMotor.mStop();
        }
        for(int i = 0; i < 20; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            Serial.print(encoderValue[i][j]);
            Serial.print(" ,");
          }
          Serial.println();
        }
        estimatePos();
        Serial.print("xpos: ");
        Serial.print(lastPos[0]);
        Serial.print(" ypos: ");
        Serial.print(lastPos[1]);
        Serial.print(" rot: ");
        Serial.println(lastPos[2]);
      }
      ctrlMotor.mStop();
      state = manualDrive;*/
      
    }
    break;
  }
}


// REMINDER! Certain parts of the libraries/makeblock/src/utility/avr/servo.cpp library was removed in order to use this
ISR(TIMER5_COMPA_vect) //short function for what happens at the interrupt
{ 
  if(encoderIndex == 200) {
    encoderIndex = 0;
  }
  Serial.println(encoderIndex);
  for(int i = 0; i < 4; i++)
  {
    encoderValue[i][encoderIndex] = ctrlMotor.getVelocity(i+1);
    Serial.print(encoderValue[i][encoderIndex]);
    Serial.print(", ");
  }
  Serial.println();
  //gyroValue[encoderIndex] = ctrlGyro.getRotation();
  encoderIndex++;
}

int wallDist() // minimum wall distance
{
  return (10 + (runCount * 5));
}

float rToM = 0.523; //constant for calculating cm/s from rpm
void estimatePos() //Function to calculate the currentposition of the robot
{
  int motorVel[200][4];
  int roboVel[200][3];
  float resPos[3] = {0,0,0};
  noInterrupts(); //there shoud be no interrupts as the encoder values are read
  //Serial.println("motorVel: ");
  for(int i = 0; i < 200; i++)
  {
    //Serial.print("index: ");
    //Serial.println(i);
    for(int j = 0; j < 2; j++) // these values are negative
    {
      if(encoderValue[i][j] == 0)
      {
        motorVel[i][j] = 0;
      }
      else
      {
        motorVel[i][j] = -encoderValue[i][j] * rToM;
       
      }
      //Serial.println(motorVel[i][j]);
    }
    for(int j = 2; j < 4; j++) // these values are positive
    {
      if(encoderValue[i][j] == 0)
      {
        motorVel[i][j] = 0;
      }
      else
      {
        motorVel[i][j] = encoderValue[i][j] * rToM;
        
      }
      //Serial.println(motorVel[i][j]);
    }
  }
  encoderIndex = 0;
  interrupts();
  //Serial.println("roboVel: ");
  for(int i = 0; i < 200; i++)
  {
    //Serial.print("index: ");
    //Serial.println(i);
    for(int j = 0; j < 4; j++)
    {
      aryEncoder[j][0] = motorVel[i][j];
    }
    calcWheelVel(dirKin, aryEncoder, aryWheel); //Calculating robot velocities from encoder velocities via direct kinematic model
    for(int j = 0; j < 3; j++)
    {
      roboVel[i][j] = aryWheel[j][0];
      //Serial.println(roboVel[i][j]);
    }
  }
  //Serial.println("resPos");
  for(int i = 0; i < 200; i++)
  {
    //Serial.print("index: ");
    //Serial.println(i);
    for(int j = 0; j < 3; j++)
    {
      //Serial.print("next: ");
      //Serial.println(j);
      if(roboVel[i][j] == 0)
      { 
        
      }
      else
      {
        //Serial.println(roboVel[i][j]*0.01);
        resPos[j] = resPos[j] + (roboVel[i][j]*0.1); //Calculating the total difference in position from the sum of the velocities times the time-interval
        
      }
      //Serial.println(resPos[j]);
    }
  }
  Serial.println("resPos: ");
  for(int i = 0; i < 3; i++)
  {
    Serial.println(resPos[i]);
    currentPos[i] = lastPos[i] + resPos[i]; 
  }

  for(int i = 0; i < 3; i++)
  {
    lastPos[i] = currentPos[i];
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
      resMat[i][j] = a/4;
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


