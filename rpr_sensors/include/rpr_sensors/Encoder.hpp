#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <errno.h>
#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

//create Encoder class
class Encoder
{

  //protected properties
  protected:
    float _countsPerRev;
    int _lastPosition;
    int _rotations;
    int _ssPin;

    //protected functions
    float readPosition();
    int getRotations();
    void setRotations(int value);

  //public properties
  public:

    //default constructors
    Encoder();
    Encoder(float countsPerRev, int ssPin);

    //basic functions
    float getCountsPerRev();
    int getSSPin();
    int getLastPosition();
    void setCountsPerRev(float cpr);
    void setLastPosition(int value);
    void setSSPin(int pin);

    //advanced functions
    float getPosition();  //returns absolute distance traveled from zero point [rotations]
    void addRotation();
    void init(float countsPerRev, int ssPin);
    void setZero();
    void subtractRotation();

};

#endif
