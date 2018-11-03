//includes
#include <Motor.hpp>

//constants
const int MIN_PULSE_DURATION = 10; //minimum delay between HIGH to LOW pulses [us]
const int MIN_PULSE_PAUSE = 10; //minimum delay between LOW to HIGH pulses [us]

Motor::Motor()
{

  //set pins to dummy values
  this->setAlarmPin(255);
  this->setDirectionPin(255);
  this->setEnablePin(255);
  this->setPulsePin(255);

}

Motor::Motor(int alarm, int direction, int enable, int pulse)
{

  //call init function to initialize object
  this->init(alarm, direction, enable, pulse);

}

//basic functions
int Motor::getAlarmPin()
{
  return this->_alarmPin;
}

int Motor::getDirectionPin()
{
  return this->_directionPin;
}

int Motor::getEnablePin()
{
  return this->_enablePin;
}

int Motor::getPulsePin()
{
  return this->_pulsePin;
}

void Motor::setAlarmPin(int pin)
{
  this->_alarmPin = pin;
}

void Motor::setDirectionPin(int pin)
{
  this->_directionPin = pin;
}

void Motor::setEnablePin(int pin)
{
  this->_enablePin = pin;
}

void Motor::setPulsePin(int pin)
{
  this->_pulsePin = pin;
}


//advanced functions
void Motor::init(int alarm, int direction, int enable, int pulse)
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

//private functions

//spins motor forward at maximum angular velocity
void Motor::pulse()
{

  //send single pulse to motor with minimum duration and pause
  digitalWrite(this->_pulsePin, HIGH);
  delayMicroseconds(MIN_PULSE_DURATION);
  digitalWrite(this->_pulsePin, LOW);
  delayMicroseconds(MIN_PULSE_PAUSE);


}

void Motor::pulse(int pulseDuration, int pulsePause)
{

  //confirm pulse duration value is valid
  if (pulseDuration < MIN_PULSE_DURATION)
    pulseDuration = MIN_PULSE_DURATION;

  //confirm pulse pause value is valid
  if (pulsePause < MIN_PULSE_PAUSE)
    pulsePause = MIN_PULSE_PAUSE;

  //send single pulse to motor with specified duration and pause
  digitalWrite(this->_pulsePin, HIGH);
  delayMicroseconds(pulseDuration);
  digitalWrite(this->_pulsePin, LOW);
  delayMicroseconds(pulsePause);

}

void Motor::setDirectionForward()
{
  digitalWrite(this->_directionPin, LOW);
}

void Motor::setDirectionReverse()
{
  digitalWrite(this->_directionPin, HIGH);
}
