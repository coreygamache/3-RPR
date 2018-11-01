//includes
#include <ball_sensor.hpp>
//#include <ros/ros.h>
#include <wiringPi.h>

/*
int _channelAPin;
int _channelBPin;
int _channelZPin;
int _clockPin;
int _misoPin;
int _mosiPin;
int _ssPin;
*/

//default constructors
Encoder::Encoder()
{

}

Encoder::Encoder(int aPin, int bPin, int zPin, int clockPin, int misoPin, int mosiPin, int ssPin)
{

}

//basic functions
int Encoder::getChannelAPin()
{
  return this->_channelAPin;
}

int Encoder::getChannelBPin()
{
  return this->_channelBPin;
}

int Encoder::getChannelZPin()
{
  return this->_channelZPin;
}

int Encoder::getClockPin()
{
  return this->_clockPin;
}

int Encoder::getMISOPin()
{
  return this->_misoPin;
}

int Encoder::getMOSIPin()
{
  return this->_mosiPin;
}

int Encoder::getSSPin()
{
  return this->_ssPin;
}

void Encoder::setChannelAPin(int pin)
{
  this->_channelAPin = pin;
}

void Encoder::setChannelBPin(int pin)
{
  this->_channelBPin = pin;
}

void Encoder::setChannelZPin(int pin)
{
  this->_channelZPin = pin;
}

void Encoder::setClockPin(int pin)
{
  this->_clockPin = pin;
}

void Encoder::setMISOPin(int pin)
{
  this->_misoPin = pin;
}

void Encoder::setMOSIPin(int pin)
{
  this->_mosiPin = pin;
}

void Encoder::setSSPin(int pin)
{
  this->_ssPin = pin;
}

//advanced functions
double Encoder::readPosition()
{

}

void Encoder::init(int aPin, int bPin, int zPin, int clockPin, int misoPin, int mosiPin, int ssPin)
{

}

void Encoder::setZero()
{

}
