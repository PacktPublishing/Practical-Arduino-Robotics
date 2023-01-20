#include "Dimmer.h"

//Instantiate a Dimmer on pin 13.
Dimmer my_dimmer(13, 500);

void setup() {
  my_dimmer.begin();
}

void loop() {
  // Generate saw-tooth brightness profile.
  int duty_cycle = millis() % 1024;
  // Set brightness with potentiometer.
  // int duty_cycle = analogRead(A0);
  my_dimmer.set_duty_cycle(duty_cycle);
  my_dimmer.blink();
}
