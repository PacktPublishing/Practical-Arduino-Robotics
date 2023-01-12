// Variables of the blink task.
unsigned long last_blink_time;
int blink_interval = 200;
// Variables of the print task.
unsigned long last_print_time;
int print_interval = 1000;
// The states of the blink task.
enum state {
  LED_OFF,
  LED_ON
};
// The blink state variable.
uint8_t led_state = LED_OFF;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize last_time variables.
  last_blink_time = millis();
  last_print_time = millis();
}

void loop() {
  // Check if it is time to run the blink task.
  if (millis() - last_blink_time >= blink_interval) {
    // Update the last blink time.
    last_blink_time += blink_interval;
    // Execute the blink task.
    blink_task();
  }

  // Check if it is time to run the print task.
  if (millis() - last_print_time >= print_interval) {
    // Update the last print time.
    last_print_time += print_interval;
    // Execute the print task.
    Serial.println("Tick");
  }
}

void blink_task() {
  switch (led_state) {
    case LED_OFF:
      // LED is off, we need to turn it on.
      digitalWrite(LED_BUILTIN, HIGH);
      // Update the state variable.
      led_state = LED_ON;
      // Update the blink interval (just for fun).
      blink_interval = 50;
      break;
    case LED_ON:
      digitalWrite(LED_BUILTIN, LOW);
      led_state = LED_OFF;
      blink_interval = 450;
      break;
  }
}