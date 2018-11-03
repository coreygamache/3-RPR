#include <EncoderMotor.hpp>
#include <wiringPi.h>

//encoder constants
const int SPI_CHANNEL = 0; //SPI bus channel (0 or 1)
const int SPI_SPEED = 500000;  //SPI bus speed (500,000 to 32,000,000)

//motor constants
const int MIN_PULSE_DURATION = 10; //minimum delay between HIGH to LOW pulses [us]
const int MIN_PULSE_PAUSE = 10; //minimum delay between LOW to HIGH pulses [us]

//default constructors
EncoderMotor::EncoderMotor()
{

  //set pins to dummy values
  this->setAlarmPin(255);
  this->setDirectionPin(255);
  this->setEnablePin(255);
  this->setPulsePin(255);
  this->setSSPin(255);

}

EncoderMotor::EncoderMotor(int alarm, int direction, int enable, int pulse, int ss)
{

  //call init function to initialize object
  this->init(alarm, direction, enable, pulse, ss);

}

//basic functions
Encoder EncoderMotor::getEncoder()
{
  return Encoder(this->_ssPin);
}

Motor EncoderMotor::getMotor()
{
  return Motor(this->_alarmPin, this->_directionPin, this->_enablePin, this->_pulsePin);
}

void EncoderMotor::setEncoder(Encoder enc)
{
  this->init(this->_alarmPin, this->_directionPin, this->_enablePin, this->_pulsePin, enc.getSSPin());
}

void EncoderMotor::setMotor(Motor motor)
{
  this->init(motor.getAlarmPin(), motor.getDirectionPin(), motor.getEnablePin(), motor.getPulsePin(), this->_ssPin);
}

//advanced functions
void EncoderMotor::init(int alarm, int direction, int enable, int pulse, int ss)
{

  //call respective init functions to initialize Encoder and Motor functionality
  this->initEncoder(ss);
  this->initMotor(alarm, direction, enable, pulse);

}

//initialize encoder functionality
void EncoderMotor::initEncoder(int ss)
{

  //set slave select pin
  this->setSSPin(ss);

  //set pin modes
  pinMode(this->_ssPin, OUTPUT);

  //setup SPI bus and display result
  int result = wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
  ROS_INFO("SPI connection result: %d", result);

  //display connection error if necessary
  if (result == -1)
  {
    ROS_INFO("error: %d", errno);
  }

}

//initialize motor functionality
void EncoderMotor::initMotor(int alarm, int direction, int enable, int pulse)
{

  //set pin variables
  this->setAlarmPin(alarm);
  this->setDirectionPin(direction);
  this->setEnablePin(enable);
  this->setPulsePin(pulse);

  //set pin modes
  pinMode(this->_alarmPin, INPUT);
  pinMode(this->_directionPin, OUTPUT);
  pinMode(this->_enablePin, OUTPUT);
  pinMode(this->_pulsePin, OUTPUT);

  //enable motor by default
  digitalWrite(this->_enablePin, HIGH);

}
