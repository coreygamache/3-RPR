//includes
#include <Encoder.hpp>

//constants
const int SPI_CHANNEL = 0; //SPI bus channel (0 or 1)
const int SPI_SPEED = 500000;  //SPI bus speed (500,000 to 32,000,000)

//default constructor
Encoder::Encoder()
{

  //set slave select pin to dummy value
  this->setSSPin(255);

}

//overloaded constructor
Encoder::Encoder(int ssPin)
{

  //call init function to initialize object
  this->init(ssPin);

}

int Encoder::getSSPin()
{
  return this->_ssPin;
}

//basic functions
void Encoder::setSSPin(int pin)
{
  this->_ssPin = pin;
}

//advanced functions
float Encoder::readPosition()
{

  //create variable for holding read/write error status
  int rwStatus = 0;

  //create buffer array for storing data
  unsigned char buffer[2];

  //set slave select pin to LOW
  digitalWrite(this->_ssPin, LOW);

  //set first byte of buffer to read position request byte
  buffer[0] = 0x10;

  //wait until encoder is ready to transmit data back
  while (buffer[0] != 0x10)
  {

    //perform simultaneous read/write operation
    rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);

    //output error if one occurred
    if (rwStatus == -1)
    {
      ROS_INFO("error: %d", errno);
      return -1.0;
    }

    //delay momentarily before retrying
    delay(2);

  }

  //receive bits
  buffer[0] = 0x00;
  buffer[1] = 0x00;
  rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);

  //set slave select pin to HIGH
  digitalWrite(this->_ssPin, HIGH);

  //mask out first 4 bits of encoder output to buffer
  buffer[0] &=~ 0xF0;

  //shift bits to produce position value from encoder output to buffer
  float position = buffer[0] << 8; //shift MSB to front of position
  position += buffer[1]; //append LSB to end of position

  //return encoder position
  return position;

}

//initialize encoder with provided slave select pin
void Encoder::init(int ssPin)
{

  //set slave select pin
  this->setSSPin(ssPin);

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

void Encoder::setZero()
{

}
