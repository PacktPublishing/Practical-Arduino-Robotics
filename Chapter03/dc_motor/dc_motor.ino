// Define the pin numbers.
const int dir_pin = 12;
const int brake_pin = 9;
const int pwm_pin = 3;
const int analog_pin = A5;

void setup() {
  // Set direction and brake pins as outputs.
  pinMode(dir_pin, OUTPUT);
  pinMode(brake_pin, OUTPUT);
  // Disable brake functionality.
  digitalWrite(brake_pin, LOW);
}

void loop() {
  // Read the potentiometer voltage.
  int analog_val = analogRead(analog_pin);
  // Map the potentiometer value to
  // direction and PWM output (see Figure 3.4).
  if (analog_val < 512) {
    // Turn forward.
    digitalWrite(dir_pin, HIGH);
    int pwm;
    pwm = map(analog_val, 0, 511, 255, 0);
    analogWrite(pwm_pin, pwm);
  } else {
    // Turn backward.
    digitalWrite(dir_pin, LOW);
    int pwm;
    pwm = map(analog_val, 512, 1023, 0, 255);
    analogWrite(pwm_pin, pwm);
  }
}