#include "MotorDriver.h"

// A simple class to use a motor driver with two direction
// and one enable (PWM) input.
// Many physical motor drivers have two such driver units.
// To use them, simply instantiate two MotorDriver objects.

// Constructor.
MotorDriver::MotorDriver(int pwm_pin, int dir_pin_A, int dir_pin_B) {
  pwm_pin_ = pwm_pin;
  dir_pin_A_ = dir_pin_A;
  dir_pin_B_ = dir_pin_B;
}

// It is safest to use hardware initiailization commands in begin() rather
// than in the constructor.
void MotorDriver::begin() {
  pinMode(dir_pin_A_, OUTPUT);
  pinMode(dir_pin_B_, OUTPUT);
  digitalWrite(dir_pin_A_, LOW);
  digitalWrite(dir_pin_B_, LOW);
  analogWrite(pwm_pin_, 0);
}

void MotorDriver::setPwm(int pwm) {
  // Constrain pwm to protect the motor.
  pwm_ = constrain(pwm, -max_pwm_, max_pwm_);
  // Set direction according to sign of pwm parameter.
  bool dir = pwm_ > 0;
  digitalWrite(dir_pin_A_, dir);
  digitalWrite(dir_pin_B_, !dir);
  int command_pwm = abs(pwm_);
  analogWrite(pwm_pin_, command_pwm);
}

void MotorDriver::adjustPwm(int val) {
  setPwm(pwm_ + val);
}

void MotorDriver::setMaxPwm(int max_pwm) {
  max_pwm_ = max_pwm;
}

int MotorDriver::getPwm() {
  return pwm_;
}
