// Begin of class definition.
class Blinker {
  // Accessible outside the class.
public:
  Blinker(int led_pin, int blink_interval) {
    // Use constructor arguments to set private variables.
    led_pin_ = led_pin;
    blink_interval_ = blink_interval;
    last_blink_time_ = millis();
  }

  // Initialize required hardware.
  void begin() {
    pinMode(led_pin_, OUTPUT);
  }

  void blink() {
    if (millis() - last_blink_time_ >= blink_interval_) {
      last_blink_time_ += blink_interval_;
      blink_task();
    }
  }

  // A public set method to change a private variable.
  void set_blink_interval(int blink_interval) {
    blink_interval_ = blink_interval;
  }

  // Only accessible within the class itself.
private:
  unsigned long last_blink_time_;
  int blink_interval_;
  int led_pin_;

  void blink_task() {
    digitalWrite(led_pin_, !digitalRead(led_pin_));
  }
};
// End of class definition.

// Instantiate a Blinker object called myBlinker.
Blinker myBlinker(13, 200);

void setup() {
  myBlinker.begin();
}

void loop() {
  myBlinker.blink();
}