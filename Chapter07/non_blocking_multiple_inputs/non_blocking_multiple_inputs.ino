#include "Blinker.h"

Blinker blinker_a(11, 50);
Blinker blinker_b(12, 50);
Blinker blinker_c(13, 50);

// States and variables of the serial parser function.
enum {
  WAITING_FOR_DATA,
  PROCESSING_DATA
};
int parser_state = WAITING_FOR_DATA;
long last_rx_data_timestamp = 0;
int rx_wait_time_ms = 10;

void setup() {
  blinker_a.begin();
  blinker_b.begin();
  blinker_c.begin();
  Serial.begin(115200);
  Serial.setTimeout(0);
}

void loop() {
  blinker_a.blink();
  blinker_b.blink();
  blinker_c.blink();
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
        // First character is the command key.
        char key = Serial.read();
        if (isDigit(Serial.peek())) {
          // After the key comes the parameter value.
          int value = Serial.parseInt();
          // Decide what to do with the value based on the key.
          switch (key) {
            case 'a':
              Serial.print("Setting blink interval a to ");
              Serial.println(value);
              blinker_a.set_blink_interval(value);
              break;
            case 'b':
              Serial.print("Setting blink interval b to ");
              Serial.println(value);
              blinker_b.set_blink_interval(value);
              break;
            case 'c':
              Serial.print("Setting blink interval c to ");
              Serial.println(value);
              blinker_c.set_blink_interval(value);
              break;
            default:
              Serial.println("Warning: Invalid key received.");
          }
        } else {
          Serial.println("Warning: Invalid value received.");
        }
        while (Serial.available() > 0) {
          Serial.read();
        }
        parser_state = WAITING_FOR_DATA;
      }
  }
}