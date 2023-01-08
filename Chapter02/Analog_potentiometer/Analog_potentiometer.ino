// Use pin A0 for analog input.
const int analog_pin = A0;

void setup() {
  // Start the Serial interface.
  Serial.begin(115200);
}

void loop() {
  // Read the potentiometer voltage.
  int analog_value = analogRead(analog_pin);
  // Print analog value every 50 ms.
  if (millis() % 50 == 0) {
    // Print analog value.
    Serial.print(analog_value);
    // Tab character creates a new line in the plotter.
    Serial.print('\t');
    // Print min and max range to avoid auto-scaling.
    Serial.print(0);
    Serial.print('\t');
    Serial.println(1023);
  }
}