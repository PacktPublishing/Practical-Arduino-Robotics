#ifndef BLINKER_H_
#define BLINKER_H_

#include "Arduino.h"

class Blinker {
public:
  Blinker(int led_pin, int blink_interval);
  void begin();
  void blink();
  void set_blink_interval(int blink_interval);

private:
  unsigned long last_blink_time_;
  int blink_interval_;
  int led_pin_;

  void blink_task();
};

#endif  // BLINKER_H_
