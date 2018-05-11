#ifndef Classes_H
#define Classes_H

#include <MeEncoderOnBoard.h>

class motor
{
  public:
    void drive(int, int, int);
    void mStop();
    void Begin();
    int getVelocity(int);
  
};

class USSens
{
 public:
  int lDist;
  int fDist;

  int getDist(bool);
  
};

class gyroSens
{
  public:
    int rot;
  
    void begin();
    int getRotation();
  
};

#endif
