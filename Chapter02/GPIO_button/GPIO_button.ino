// We declare pin numbers as constants.
const int button_pin = 2;
// We declare ISR variables volatile.
volatile unsigned long num_button_events = 0;

// Interrupt service routine.
void buttonIsr() {
  num_button_events = num_button_events + 1;
}

void setup() {
  // Start the Serial interface.
  Serial.begin(115200);
  // Set LED pin as output.
  pinMode(LED_BUILTIN, OUTPUT);
  // Set button pin as input. Activate pull-up.
  pinMode(button_pin, INPUT_PULLUP);
  // Attach the ISR to the interrupt source:
  // Any change of the state of button_pin.
  attachInterrupt(digitalPinToInterrupt(button_pin), buttonIsr, CHANGE);
}

void loop() {
  // Read the button state (HIGH or LOW).
  bool button_state = digitalRead(button_pin);
  // Control the LED accordingly. 
  // Invert input to turn LED ON when input is LOW.
  digitalWrite(LED_BUILTIN, !button_state);
  // Periodically print number of state changes.
  if (millis() % 100 == 0) {
    Serial.println(num_button_events);
  }
}
