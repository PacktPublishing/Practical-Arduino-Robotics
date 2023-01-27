#include "QuadEncoder.h"

QuadEncoder::QuadEncoder(int pin_a, int pin_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  last_velocity_sampling_time_ = micros();
}

void QuadEncoder::changeIsrA() {
  switch (state_a_) {
  case LOW:  // This was a rising event.
    risingA();
    break;
  case HIGH:  // This was a falling event.
    fallingA();
    break;
  }
}

void QuadEncoder::changeIsrB() {
  switch (state_b_) {
  case LOW:  // This was a rising event.
    risingB();
    break;
  case HIGH:  // This was a falling event.
    fallingB();
    break;
  }
}

int QuadEncoder::getPinA() {
  return pin_a_;
}

int QuadEncoder::getPinB() {
  return pin_b_;
}

long QuadEncoder::getPosition() {
  return position_;
}

float QuadEncoder::getVelocity() {
  return velocity_cps_;
}

float QuadEncoder::getRawVelocity() {
  return velocity_raw_;
}

void QuadEncoder::begin() {
  pinMode(pin_a_, INPUT);
  pinMode(pin_b_, INPUT);
  state_a_ = digitalRead(pin_a_);
  state_b_ = digitalRead(pin_b_);
}

void QuadEncoder::update() {
  unsigned long now = micros();
  if (now >= (last_velocity_sampling_time_ + kSamplingIntervalUs_)) {
    // It is time to update the velocity estimate.
    // Update last_sampling_time_.
    last_velocity_sampling_time_ += kSamplingIntervalUs_;
    // Retrieve the oldest position in the sliding window.
    oldest_position_ = position_window_[sampling_index_];  
    // Overwrite the oldest position with the newest. 
    position_window_[sampling_index_] = position_;
    // Increment the sampling index. This is what makes the window slide.
    // Modulo handles the wrapping of the ring buffer.    
    sampling_index_ = (sampling_index_ + 1) % kWindowSize_;  
    // Compute velocity estimate as average velocity over the sliding window.
    velocity_cps_ = 1000000.0 * (float) (position_ - oldest_position_) /
                    ((float) (kSamplingIntervalUs_ * kWindowSize_));
    // Update unfiltered velocity estimate. Just for comparison.
    int positon_delta = position_ - last_position_;
    velocity_raw_ = 1000000.0 * (float) (position_ - last_position_) /
                    ((float) kSamplingIntervalUs_);
    // Update last position.
    last_position_ = position_;
  }
}

// These functions are used inside the interrupt handlers.
void QuadEncoder::risingA() {
  state_a_ = HIGH;
  switch (state_b_) {
  case LOW:
    position_++;
    break;
  case HIGH:
    position_--;
    break;
  }
}

void QuadEncoder::fallingA() {
  state_a_ = LOW;
  switch (state_b_) {
  case LOW:
    position_--;
    break;
  case HIGH:
    position_++;
    break;
  }
}

void QuadEncoder::risingB() {
  state_b_ = HIGH;
  switch (state_a_) {
  case LOW:
    position_--;
    break;
  case HIGH:
    position_++;
    break;
  }
}

void QuadEncoder::fallingB() {
  state_b_ = LOW;
  switch (state_a_) {
  case LOW:
    position_++;
    break;
  case HIGH:
    position_--;
    break;
  }
}
