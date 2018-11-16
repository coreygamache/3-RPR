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
  this->setLastPosition(-1);
  this->setRotations(0);
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

int Encoder::getLastPosition()
{
  return this->_lastPosition;
}

int Encoder::getRotations()
{
  return this->_rotations;
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

void Encoder::setLastPosition(int value)
{

  //check value is valid before setting
  if (value > this->_countsPerRev)
    value = this->_countsPerRev;
  else if (value < 0)
    value = 0;

  //set last position to checked value
  this->_lastPosition = value;

}

void Encoder::setRotations(int value)
{
  this->_rotations = value;
}

void Encoder::setSSPin(int pin)
{
  this->_ssPin = pin;
}

//advanced functions

void Encoder::addRotation()
{

  //add rotation if new value will be valid, otherwise wrap to zero to prevent overflow
  if (this->_rotations < 2147483647)
    this->_rotations++;
  else
    this->_rotations = 0;

}

//return distance traveled [rotations]
float Encoder::getPosition()
{

  //read current position and convert to rotations
  float distanceTraveled = this->readPosition() / this->_countsPerRev;

  //add number of rotations already traveled to total distance travel
  distanceTraveled += this->_rotations;

  //return total distance traveled
  return distanceTraveled;

}

//initialize encoder with provided slave select pin
void Encoder::init(float countsPerRev, int ssPin)
{

  //run standard wiringPi setup routine
  wiringPiSetup();

  //set counts per revolution and slave select pin
  this->setCountsPerRev(countsPerRev);
  this->setLastPosition(-1);
  this->setRotations(0);
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

float Encoder::readPosition()
{

  //create buffer array for storing data
  unsigned char buffer[2];

  //set first byte of buffer to read position request byte
  buffer[0] = 0x10;

  //set slave select pin to LOW
  digitalWrite(this->_ssPin, LOW);

  //perform simultaneous read/write operation
  int rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);

  //set slave select pin to HIGH
  digitalWrite(this->_ssPin, HIGH);

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

    //set slave select pin to LOW
    digitalWrite(this->_ssPin, LOW);

    //perform simultaneous read/write operation
    rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);

    //set slave select pin to HIGH
    digitalWrite(this->_ssPin, HIGH);

    //output error if one occurred
    if (rwStatus == -1)
    {
      ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);
      return -1.0;
    }

    //delay momentarily before retrying
    delay(2);

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

  //set slave select pin to LOW
  digitalWrite(this->_ssPin, LOW);
  rwStatus = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);

  //set slave select pin to HIGH
  digitalWrite(this->_ssPin, HIGH);

  if (rwStatus == -1)
    ROS_ERROR("[ERROR] error reading/writing via SPI bus: %d", errno);

  //mask out first 4 bits of encoder output to buffer
  buffer[0] &=~ 0xF0;

  //shift bits to produce position value from encoder output to buffer
  float position = buffer[0] << 8; //shift MSB to front of position
  position += buffer[1]; //append LSB to end of position

  //increment rotations counter if a rotation has been completed
  if ((this->_lastPosition != -1) && (this->_lastPosition > (this->_countsPerRev * 0.9)) && (position < this->_countsPerRev * 0.1))
    this->addRotation();
  else if ((this->_lastPosition != -1) && (this->_lastPosition < (this->_countsPerRev * 0.1)) && (position > this->_countsPerRev * 0.9))
    this->subtractRotation();

  //return encoder position
  return position;

}

void Encoder::setZero()
{

}

void Encoder::subtractRotation()
{

  //subtract rotation if value will be valid, otherwise wrap to maximum to prevent overflow
  if (this->_rotations > 0)
    this->_rotations--;
  else
    this->_rotations = 2147483647;

}
