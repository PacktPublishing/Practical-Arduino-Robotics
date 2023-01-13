#include "Blinker.h"

Blinker myBlinker(13, 50);

void setup() {
  myBlinker.begin();
}

void loop() {
  myBlinker.blink();
}
