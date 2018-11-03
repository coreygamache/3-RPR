#ifndef MOTOR_HPP
#define MOTOR_HPP

//create Motor class
class Motor
{

  //private properties
  private:
    int _alarmPin;
    int _directionPin;
    int _enablePin;
    int _pulsePin;

    //private functions
    void pulse();
    void pulse(int pulseDuration, int pulsePause);

  //public properties
  public:

    //default constructors
    Motor();
    Motor(int alarm, int direction, int enable, int pulse);

    //basic functions
    int getAlarmPin();
    int getDirectionPin();
    int getEnablePin();
    int getPulsePin();
    void setAlarmPin(int pin);
    void setDirectionPin(int pin);
    void setEnablePin(int pin);
    void setPulsePin(int pin);

    //advanced functions
    void init(int alarm, int direction, int enable, int pulse);

};

#endif
