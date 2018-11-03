#ifndef ENCODER_HPP
#define ENCODER_HPP

//create Encoder class
class Encoder
{

  //private properties
  private:
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
