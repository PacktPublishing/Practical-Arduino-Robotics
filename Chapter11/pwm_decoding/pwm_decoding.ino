volatile unsigned long rise_time_us;
volatile unsigned int pulse_width_us;
const short pwm_input_pin = 2;

void setup() {
  Serial.begin(115200);
  // Attach interrupt handler to rising edge.
  attachInterrupt(
    digitalPinToInterrupt(pwm_input_pin), 
    risingISR, RISING);
}

void loop() {
  Serial.println(pulse_width_us);
  delay(20);
}

void risingISR() {
  // Pulse started. Note the start time.
  rise_time_us = micros();
  // Attach interrupt handler to falling edge.
  attachInterrupt(
    digitalPinToInterrupt(pwm_input_pin), 
    fallingISR, FALLING);
}

void fallingISR() {
  // Pulse is over. Compute pulse width.
  pulse_width_us = micros() - rise_time_us;
  // Attach interrupt handler to rising edge.
  attachInterrupt(
    digitalPinToInterrupt(pwm_input_pin), 
    risingISR, RISING);
}
