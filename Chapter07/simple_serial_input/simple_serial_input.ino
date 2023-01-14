// Include your custom Blinker library.
#include "Blinker.h"

Blinker myBlinker(13, 50);

void setup() {
  myBlinker.begin();
  Serial.begin(115200);
  Serial.setTimeout(0);
}

void loop() {
  myBlinker.blink();
  parse_interval();
}

// Read Serial input and interpret it as blink interval.
void parse_interval() {
  int new_interval = 0;
  if (Serial.available() > 0) {
    // Wait to receive the entire message.
    delay(10);  
    if (isDigit(Serial.peek())) {
      new_interval = Serial.parseInt();
      Serial.print("Setting blink interval to ");
      Serial.println(new_interval);
      myBlinker.set_blink_interval(new_interval);
    } else {
      Serial.println("Warning: Invalid data received.");
    }
    while (Serial.available() > 0) {
      // Flush the serial input buffer.
      Serial.read();
    }
  }
}