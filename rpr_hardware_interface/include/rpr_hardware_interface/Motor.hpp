#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <wiringPi.h>

//create Motor class
class Motor
{

  //private properties
  protected:
    float _max_rpm;
    float _min_high_pulse_width;    //minimum delay between HIGH to LOW pulses [us]
    float _min_low_pulse_width;     //minimum delay between LOW to HIGH pulses [us]
    int _alarmPin;
    int _directionPin;
    int _enablePin;
    int _pulsePin;

  //public properties
  public:

    //default constructors
    Motor();
    Motor(int alarm, int direction, int enable, int pulse, float max_rpm, float min_high_pulse_width, float min_low_pulse_width);

    //basic functions
    float getMaxRPM();
    float getMinHighPulseWidth();
    float getMinLowPulseWidth();
    int getAlarmPin();
    int getDirectionPin();
    int getEnablePin();
    int getPulsePin();
    void setAlarmPin(int pin);
    void setDirectionPin(int pin);
    void setEnablePin(int pin);
    void setMaxRPM(float value);
    void setMinHighPulseWidth(float value);
    void setMinLowPulseWidth(float value);
    void setPulsePin(int pin);

    //advanced functions
    void init(int alarm, int direction, int enable, int pulse, float max_rpm, float min_high_pulse_width, float min_low_pulse_width);
    void pulse();
    void pulse(int pulseDuration, int pulsePause);
    void setDirectionForward();
    void setDirectionReverse();

};

#endif
