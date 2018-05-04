#ifndef Classes_H
#define Classes_H

class motor
{
  public:
    void drive(int, int, int);
    void mStop();
    void Begin();
    void getVelocities(int[4]);
  
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
