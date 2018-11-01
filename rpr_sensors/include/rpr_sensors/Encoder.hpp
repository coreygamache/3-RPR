#ifndef ENCODER_HPP
#define ENCODER_HPP

//create Encoder class
class Encoder
{

  //private properties
  private:
    int _channelAPin;
    int _channelBPin;
    int _channelZPin;
    int _clockPin;
    int _misoPin;
    int _mosiPin;
    int _ssPin;

  //public properties
  public:

    //default constructors
    Encoder();
    Encoder(int aPin, int bPin, int zPin, int clockPin, int misoPin, int mosiPin, int ssPin);

    //basic functions
    int getChannelAPin();
    int getChannelBPin();
    int getChannelZPin();
    int getClockPin();
    int getMISOPin();
    int getMOSIPin();
    int getSSPin();
    void setChannelAPin(int pin);
    void setChannelBPin(int pin);
    void setChannelZPin(int pin);
    void setClockPin(int pin);
    void setMISOPin(int pin);
    void setMOSIPin(int pin);
    void setSSPin(int pin);

    //advanced functions
    double readPosition();
    void init(int aPin, int bPin, int zPin, int clockPin, int misoPin, int mosiPin, int ssPin);
    void setZero();

};

#endif
