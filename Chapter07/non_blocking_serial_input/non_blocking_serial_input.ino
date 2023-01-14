#include "Blinker.h"

Blinker myBlinker(13, 50);

// States and variables of the serial parser function.
enum {
  WAITING_FOR_DATA,
  PROCESSING_DATA
};
int parser_state = WAITING_FOR_DATA;
long last_rx_data_timestamp = 0;
int rx_wait_time_ms = 10;

void setup() {
  myBlinker.begin();
  Serial.begin(115200);
  Serial.setTimeout(0);
}

void loop() {
  myBlinker.blink();
  parse_interval();
}

// Non-blocking version of parse_interval().
void parse_interval() {
  switch (parser_state) {
    case WAITING_FOR_DATA:
      if (Serial.available() > 0) {
        // Data received.
        last_rx_data_timestamp = millis();
        parser_state = PROCESSING_DATA;
      }
      break;
    case PROCESSING_DATA:
      if (millis() - last_rx_data_timestamp >= rx_wait_time_ms) {
        // Enough time has passed since the first byte was received.
        if (isDigit(Serial.peek())) {
          int new_interval = Serial.parseInt();
          Serial.print("Setting blink interval to ");
          Serial.println(new_interval);
          myBlinker.set_blink_interval(new_interval);
        } else {
          Serial.println("Warning: Invalid data received.");
        }
        while (Serial.available() > 0) {
          Serial.read();
        }
        // Reset parser state.
        parser_state = WAITING_FOR_DATA;  
      }
  }
}