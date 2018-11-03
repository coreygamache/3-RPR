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
    int _ssPin;

  //public properties
  public:

    //default constructors
    Encoder();
    Encoder(int ssPin);

    //basic functions
    int getSSPin();
    void setSSPin(int pin);

    //advanced functions
    float readPosition();
    void init(int ssPin);
    void setZero();

};

#endif
