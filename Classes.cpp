#include "Classes.h"
#include <MeMegaPi.h>

MeEncoderOnBoard motor1(SLOT1); //Front left
MeEncoderOnBoard motor2(SLOT2); //Back left
MeEncoderOnBoard motor3(SLOT3); //Back right
MeEncoderOnBoard motor4(SLOT4); //Front right

MeUltrasonicSensor ultraSensorL(PORT_7); //Left Sensor
MeUltrasonicSensor ultraSensorF(PORT_6); //Front Sensor

MeGyro gyroSensor;


// Functions to initialise encoders
void isr_process_encoder1(void)
{
  if(digitalRead(motor1.getPortB()) == 0)
  {
    motor1.pulsePosMinus();
  }
  else
  {
    motor1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(motor2.getPortB()) == 0)
  {
    motor2.pulsePosMinus();
  }
  else
  {
    motor2.pulsePosPlus();
  }
}
void isr_process_encoder3(void)
{
  if(digitalRead(motor3.getPortB()) == 0)
  {
    motor3.pulsePosMinus();
  }
  else
  {
    motor3.pulsePosPlus();
  }
}
void isr_process_encoder4(void)
{
  if(digitalRead(motor4.getPortB()) == 0)
  {
    motor4.pulsePosMinus();
  }
  else
  {
    motor4.pulsePosPlus();
  }
}

// Function for calculating wheel velocities
void getMotorVel(int matA[4][3], int matB[3][1], int resMat[4][1])
{
  double a = 0;
  for(int i = 0; i < 4;i++)
  {
    for(int j = 0; j < 1; j++)
    {
      int a = 0;
      for(int k = 0; k < 3; k++)
      {
        a += matA[i][k] * matB[k][j];
      }
      resMat[i][j] = a;
    }
  }  
}

// d+l = 19
// Constant multiplication array for calculation wheel velocities
int aryA[4][3] = { {1, -1, -(19)},
                   {1, 1, -(19)},
                   {1, -1, (19)},
                   {1, 1, (19)}
                 };

// Array of wanted velocities, xdot, ydot, phidot
int aryIn[3][1] = { {0},
                   {0},
                   {0}
                 };
// Array of resulting wheel velocities
int aryVel[4][1];


float rToM = 0.523; //constant for calculating m/s from rpm
float mToR = 2.63; //constant from calculation rpm from m/s

//motor class memberfunctions

//Initialising encoders
void motor::Begin()
{
  attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(motor3.getIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(motor4.getIntNum(), isr_process_encoder4, RISING);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);  
}

//function that makes to robot drive. input: x-velocity, y-veloticy, rotational velocity
void motor::drive(int vX, int vY, int vPhi)
{
  //assigning velocities to array
  aryIn[0][0] = vX;
  aryIn[1][0] = vY;
  aryIn[2][0] = vPhi;
  //Calculating wheel velocities from robot velocities
  getMotorVel(aryA,aryIn,aryVel);
 
  //Applying calculated velocities to each motor multiplying with constant mToR to get motor indput 0-255
  motor1.setMotorPwm(-aryVel[0][0] * mToR);
  motor2.setMotorPwm(-aryVel[1][0] * mToR);
  motor3.setMotorPwm(aryVel[2][0] * mToR);
  motor4.setMotorPwm(aryVel[3][0] * mToR);
  motor1.updateSpeed();
  motor2.updateSpeed();
  motor3.updateSpeed();
  motor4.updateSpeed();
}

//stopping all motors
void motor::mStop()
{
  motor1.setMotorPwm(0);
  motor2.setMotorPwm(0);
  motor3.setMotorPwm(0);
  motor4.setMotorPwm(0);
  motor1.updateSpeed();
  motor2.updateSpeed();
  motor3.updateSpeed();
  motor4.updateSpeed();
}

void motor::getVelocities(int ary[4])
{
  ary[0] = motor1.getCurrentSpeed();
  ary[1] = motor2.getCurrentSpeed();
  ary[2] = motor3.getCurrentSpeed();
  ary[3] = motor4.getCurrentSpeed();
}

//Ultrasonicsensor class memberfunctions

//Function for getting current distances from sensors
void USSens::getDist()
{
  lDist = ultraSensorL.distanceCm();
  fDist = ultraSensorF.distanceCm();
}

// Gyrosensor class memberfunctions

//Function to initialise gyro sensor
void gyroSens::begin()
{
  gyroSensor.begin();
}

//Function to get current rotation of the robot in the x-y plane
void gyroSens::getRotation()
{
  gyroSensor.update();
  rot = gyroSensor.getGyroZ();
}




