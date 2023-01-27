#include "Arduino.h"

#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H

class MotorDriver {
  public:
    // Constructor.
    MotorDriver(int pwm_pin, int dir_pin_A, int dir_pin_B);  
    // Initialization. Call inside setup().
    void begin();
    // Set PWM value. Negative values reverse direction.
    void setPwm(int pwm);
    // Set the maximum admissible PWM value.
    void setMaxPwm(int max_pwm);
     // Change PWM value by a certain value.
    void adjustPwm(int val);
    // Get the current PWM value.
    int getPwm();

  private:
    int pwm_pin_ = 0;
    int dir_pin_A_ = 0;
    int dir_pin_B_ = 0;
    int pwm_ = 0;
    int max_pwm_ = 255;
};

#endif  // _MOTOR_DRIVER_H
