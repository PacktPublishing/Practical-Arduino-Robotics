#include "Arduino.h"

#ifndef _QUAD_ENCODER_H
#define _QUAD_ENCODER_H

class QuadEncoder {
public:
  QuadEncoder(int pin_a, int pin_b);

  // These Interrupt service handlers are meant to be attached to
  // CHANGE events on the Arduino's hardware interrupts.
  // You need to create global wrappers around them in the main program
  // to do that.
  void changeIsrA();
  void changeIsrB();

  int getPinA();
  int getPinB();
  long getPosition();
  float getVelocity();
  float getRawVelocity();

  // Function to initialize the state of the two inputs.
  void begin();

  // Function to update the velocity estimate.
  // We cannot measure the velocity directly, so we need to estimate it. Using
  // a sliding window average approach gives a nice and tunable trade-off
  // between lag and noise. This non-blocking update function needs to be
  // called as often as possible.
  void update();

private:
  // These two settings determine the characteristics of the velocity
  // estimate. Higher numbers will result in a smoother estimate but also
  // create more lag.
  static const unsigned long kWindowSize_ = 80;
  static const unsigned long kSamplingIntervalUs_ = 4000;

  long position_ = 0;
  long last_position_ = 0;  // Only needed for unfiltered velocity estimation.
  int pin_a_ = 0;
  int pin_b_ = 0;
  bool state_a_ = LOW;
  bool state_b_ = LOW;
  long int oldest_position_ = 0;
  unsigned long last_velocity_sampling_time_ = 0;
  float velocity_cps_ = 0;
  float velocity_raw_ = 0;  // Only needed for unfiltered velocity estimation.
  long int position_window_[kWindowSize_];
  uint8_t sampling_index_ = 0;

  // These functions are used inside the interrupt handlers.
  void risingA();
  void fallingA();
  void risingB();
  void fallingB();
};

#endif  // _QUAD_ENCODER_H
