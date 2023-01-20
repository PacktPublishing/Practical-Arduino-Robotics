#ifndef DIMMER_H_
#define DIMMER_H_

#include "Arduino.h"

class Dimmer {
public:
  Dimmer(int led_pin, int duty_cycle);

  void begin();

  void blink();

  void set_duty_cycle(int duty_cycle);

private:
  enum blink_state {
    LED_OFF,
    LED_ON
  };
  unsigned long last_blink_time_ = 0;
  int blink_interval_ = 0;  // microseconds
  int led_pin_ = 0;
  uint8_t led_state_ = LED_OFF;
  int duty_cycle_ = 0;  // microseconds

  void blink_task();
};

#endif DIMMER_H_
