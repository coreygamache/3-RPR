#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <errno.h>
#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

//create Encoder class
class Encoder
{

  //private properties
  protected:
    float _countsPerRev;
    int _ssPin;

  //public properties
  public:

    //default constructors
    Encoder();
    Encoder(float countsPerRev, int ssPin);

    //basic functions
    float getCountsPerRev();
    int getSSPin();
    void setCountsPerRev(float cpr);
    void setSSPin(int pin);

    //advanced functions
    float readPosition();
    void init(float countsPerRev, int ssPin);
    void setZero();

};

#endif
