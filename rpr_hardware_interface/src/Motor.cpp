//includes
#include <Motor.hpp>

Motor::Motor()
{

  //run standard wiringPi setup routine
  wiringPiSetup();

  //set pins to dummy values
  this->setAlarmPin(255);
  this->setDirectionPin(255);
  this->setEnablePin(255);
  this->setPulsePin(255);

  //set other variables to dummy values
  this->setMaxRPM(1);
  this->setMinHighPulseWidth(2.5);
  this->setMinLowPulseWidth(2.5);

}

Motor::Motor(int alarm, int direction, int enable, int pulse, float max_rpm, float min_high_pulse_width, float min_low_pulse_width)
{

  //call init function to initialize object
  this->init(alarm, direction, enable, pulse, max_rpm, min_high_pulse_width, min_low_pulse_width);

}

//basic functions
float Motor::getMaxRPM()
{
  return this->_max_rpm;
}

float Motor::getMinHighPulseWidth()
{
  return this->_min_high_pulse_width;
}

float Motor::getMinLowPulseWidth()
{
  return this->_min_low_pulse_width;
}

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

void Motor::setMaxRPM(float value)
{
  this->_max_rpm = value;
}

void Motor::setMinHighPulseWidth(float value)
{
  this->_min_high_pulse_width = value;
}

void Motor::setMinLowPulseWidth(float value)
{
  this->_min_low_pulse_width = value;
}

void Motor::setPulsePin(int pin)
{
  this->_pulsePin = pin;
}

//advanced functions
void Motor::init(int alarm, int direction, int enable, int pulse, float max_rpm, float min_high_pulse_width, float min_low_pulse_width)
{

  //run standard wiringPi setup routine
  wiringPiSetup();

  //set pin variables
  this->setAlarmPin(alarm);
  this->setDirectionPin(direction);
  this->setEnablePin(enable);
  this->setPulsePin(pulse);

  //set other variables
  this->setMaxRPM(max_rpm);
  this->setMinHighPulseWidth(min_high_pulse_width);
  this->setMinLowPulseWidth(min_low_pulse_width);

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
  delayMicroseconds(this->_min_high_pulse_width);
  digitalWrite(this->_pulsePin, LOW);
  delayMicroseconds(this->_min_low_pulse_width);


}

void Motor::pulse(int pulseDuration, int pulsePause)
{

  //confirm pulse duration value is valid
  if (pulseDuration < this->_min_high_pulse_width)
    pulseDuration = this->_min_high_pulse_width;

  //confirm pulse pause value is valid
  if (pulsePause < this->_min_low_pulse_width)
    pulsePause = this->_min_low_pulse_width;

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
