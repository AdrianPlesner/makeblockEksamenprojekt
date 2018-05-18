#ifndef Classes_H
#define Classes_H

class motor
{
  public:
  //motor class memberfunctions
    void Begin(); //initialise encoders
    void drive(int, int, int); // drive takses 3 arguments, Xm, Ym, PHIm
    void mStop(); //stop all motors
    int getVelocity(int); // get the encoder velocity, 1 argument = motor number, returns the velocity of that motor
  
};

class USSens
{
 public:
 //ultrasouns sensor class member variables
  int lDist; // distance of left sensor
  int fDist; // distance of front sensor
  //ultrasound sensor class memberfunctions
  int getDist(bool); // get the distance of a sensor, 1 argument true for front, false for left. returns the distance for that sensor
  
};

class gyroSens
{
  public:
    //gyrosensor class member variables
    int rot;
    //gyrosensor class memberfunctions
    void begin(); //initialise gyrosensor
    int getRotation(); // returns the current angleposition
  
};

#endif
