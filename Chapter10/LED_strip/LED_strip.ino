#include "Dimmer.h"

// Transistor for red LEDs connected to pin 2.
Dimmer redDimmer(2, 100);
// Transistor for green LEDs connected to pin 3.
Dimmer greenDimmer(3, 100);
// Transistor for blue LEDs connected to pin 4.
Dimmer blueDimmer(4, 100);

// Variables for Serial input parsing.
enum {
  WAITING_FOR_DATA,
  PROCESSING_DATA
};

int parser_state = WAITING_FOR_DATA;
long last_rx_data_timestamp = 0;
int rx_wait_time_ms = 10;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  redDimmer.begin();
  greenDimmer.begin();
  blueDimmer.begin();
}

void loop() {
  redDimmer.blink();
  greenDimmer.blink();
  blueDimmer.blink();
  // Set brightness levels via Serial input.
  parse_interval();
  // Or set brigthness levels via analog inputs.
  // redDimmer.set_duty_cycle(analogRead(A0));
  // greenimmer.set_duty_cycle(analogRead(A1));
  // blueDimmer.set_duty_cycle(analogRead(A2));
}

void parse_interval() {
  switch (parser_state) {
    case WAITING_FOR_DATA:
      if (Serial.available() > 0) {
        last_rx_data_timestamp = millis();
        parser_state = PROCESSING_DATA;
      }
      break;
    case PROCESSING_DATA:
      if (millis() - last_rx_data_timestamp >= rx_wait_time_ms) {
        char key = Serial.read();
        if (isDigit(Serial.peek())) {
          int value = Serial.parseInt();
          switch (key) {
            case 'r':
              redDimmer.set_duty_cycle(value);
              break;
            case 'g':
              greenDimmer.set_duty_cycle(value);
              break;
            case 'b':
              blueDimmer.set_duty_cycle(value);
              break;
            default:
              Serial.println("Invalid key received. Clearing all colors.");
              redDimmer.set_duty_cycle(0);
              greenDimmer.set_duty_cycle(0);
              blueDimmer.set_duty_cycle(0);
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
