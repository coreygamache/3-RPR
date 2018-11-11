//includes
#include <Encoder.hpp>

//constants
const int SPI_CHANNEL = 0; //SPI bus channel (0 or 1)
const int SPI_SPEED = 500000;  //SPI bus speed (500,000 to 32,000,000)

//default constructor
Encoder::Encoder()
{

  //run standard wiringPi setup routine
  wiringPiSetup();

  //set slave select pin and counts per revolution to dummy values
  this->setCountsPerRev(1.0);
  this->setSSPin(255);

}

//overloaded constructor
Encoder::Encoder(float countsPerRev, int ssPin)
{

  //call init function to initialize object
  this->init(countsPerRev, ssPin);

}

//basic functions
float Encoder::getCountsPerRev()
{
  return this->_countsPerRev;
}

int Encoder::getSSPin()
{
  return this->_ssPin;
}

void Encoder::setCountsPerRev(float countsPerRev)
{

  //verify counts per revolution value is valid
  if (countsPerRev <= 0)
    countsPerRev = 1.0;

  //set object counts per revolution variable to valid provided value
  this->_countsPerRev = countsPerRev;

}

void Encoder::setSSPin(int pin)
{
  this->_ssPin = pin;
}

//advanced functions
float Encoder::readPosition()
{

  //create buffer array for storing data
  unsigned char buffer[2];

  //set slave select pin to LOW
  digitalWrite(this->_ssPin, LOW);

  //set first byte of buffer to read position request byte
  buffer[0] = 0x10;

  //perform simultaneous read/write operation
  int rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);

  //output error if one occurred
  if (rwStatus == -1)
  {
    ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);
    return -1.0;
  }

  //wait until encoder is ready to transmit data back
  while (buffer[0] != 0x10)
  {

    //reset buffer to issue NOP command
    buffer[0] = 0x00;

    //perform simultaneous read/write operation
    rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);

    ROS_INFO("current reading: %d", buffer[0]);

    //output error if one occurred
    if (rwStatus == -1)
    {
      ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);
      return -1.0;
    }

    //delay momentarily before retrying
    delay(250);

  }

  //receive bits
  /*for (int i = 0; i < 2; i++)
  {
    buffer[i] = 0x00;
    rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, &buffer[i], 1);

    if (rwStatus == -1)
      ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);
  }*/
  buffer[0] = 0x00;
  buffer[1] = 0x00;
  rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);

  if (rwStatus == -1)
    ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);

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
void Encoder::init(float countsPerRev, int ssPin)
{

  //run standard wiringPi setup routine
  wiringPiSetup();

  //set counts per revolution and slave select pin
  this->setCountsPerRev(countsPerRev);
  this->setSSPin(ssPin);

  //set pin modes and enable pullup resistor
  pinMode(this->_ssPin, OUTPUT);
  digitalWrite(this->_ssPin, HIGH);

  //setup SPI bus and display result
  int result = wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
  ROS_INFO("SPI connection result: %d", result);

  //display connection error if necessary
  if (result == -1)
    ROS_ERROR("[ERROR] error connecting to SPI bus: %d", errno);

}

void Encoder::setZero()
{

}
