// Variables of the blink task.
unsigned long last_blink_time;
int blink_interval = 200;
// Variables of the print task.
unsigned long last_print_time;
int print_interval = 1000;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize last_time variables.
  last_blink_time = millis();
  last_print_time = millis();
}

void loop() {
  // Input for variable blink frequency.
  // blink_interval = analogRead(A0);
  // Check if it is time to run the blink task.
  if (millis() - last_blink_time >= blink_interval) {
    // Update the last blink time.
    last_blink_time += blink_interval;
    // Execute the blink task.
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Check if it is time to run the print task.
  if (millis() - last_print_time >= print_interval) {
    // Update the last print time.
    last_print_time += print_interval;
    // Execute the print task.
    Serial.println("Tick");
  }
}