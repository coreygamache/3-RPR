#ifndef ENCODERMOTOR_HPP
#define ENCODERMOTOR_HPP

#include <Encoder.hpp>
#include <Motor.hpp>

/* ------------------------
inhereted private variables

int _alarmPin;
int _directionPin;
int _enablePin;
int _pulsePin;
int _ssPin;
------------------------ */

//create Encoder class
class EncoderMotor: public Encoder, public Motor
{

  //private properties
  private:


  //public properties
  public:

    //default constructors
    EncoderMotor();
    EncoderMotor(int alarm, int direction, int enable, int pulse, int ss);

    //basic functions
    Encoder getEncoder();
    Motor getMotor();
    void setEncoder(Encoder enc);
    void setMotor(Motor motor);

    //advanced functions
    void init(int alarm, int direction, int enable, int pulse, int ss);
    void initEncoder(int ss);
    void initMotor(int alarm, int direction, int enable, int pulse);

};

#endif
