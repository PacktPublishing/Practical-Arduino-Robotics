#include "Dimmer.h"

Dimmer::Dimmer(int led_pin, int duty_cycle) {
  led_pin_ = led_pin;
  blink_interval_ = 1024;  // microseconds
  duty_cycle_ = duty_cycle;
}

void Dimmer::begin() {
  pinMode(led_pin_, OUTPUT);
}

void Dimmer::blink() {
  if (micros() - last_blink_time_ >= blink_interval_) {
    blink_task();
  }
}

void Dimmer::set_duty_cycle(int duty_cycle) {
  duty_cycle_ = constrain(duty_cycle, 0, 1023);
}


void Dimmer::blink_task() {
  switch (led_state_) {
    case LED_OFF:
      if (duty_cycle_ > 0) {
        digitalWrite(led_pin_, HIGH);
      }
      led_state_ = LED_ON;
      // Will be switched off after duty cycle has elapsed.
      last_blink_time_ += duty_cycle_;
      break;
    case LED_ON:
      digitalWrite(led_pin_, LOW);
      led_state_ = LED_OFF;
      last_blink_time_ += blink_interval_ - duty_cycle_;
      break;
  }
}
