#include "Blinker.h"

Blinker::Blinker(int led_pin, int blink_interval) {
  led_pin_ = led_pin;
  blink_interval_ = blink_interval;
  last_blink_time_ = millis();
}

void Blinker::begin() {
  pinMode(led_pin_, OUTPUT);
}

void Blinker::blink() {
  if (millis() - last_blink_time_ >= blink_interval_) {
    last_blink_time_ += blink_interval_;
    blink_task();
  }
}

void Blinker::set_blink_interval(int blink_interval) {
  blink_interval_ = blink_interval;
}

void Blinker::blink_task() {
  digitalWrite(led_pin_, !digitalRead(led_pin_));
}
