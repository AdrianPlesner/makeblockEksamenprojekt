#ifndef Classes_H
#define Classes_H

#include <MeEncoderOnBoard.h>

class motor
{
  public:
    void drive(int, int, int);
    void mStop();
    void Begin();
    float getVelocity(int);
  
};

class USSens
{
 public:
  int lDist;
  int fDist;

  void getDist();
  
};

class gyroSens
{
  public:
    int rot;
  
    void begin();
    void getRotation();
  
};

#endif
