// Include the Servo library.
#include <Servo.h>

// Analog input pin.
const int input_pin = A5;
// Servo output pin. Must be a PWM pin.
const int servo_pin = 3;
// Instantiate a Servo object called servo.
Servo servo;

void setup() {
  // Attach the servo object to the servo pin.
  servo.attach(servo_pin);
}

void loop() {
  // Read the potentiometer.
  int input = analogRead(input_pin);
  // Map the analog value from 0 to 1023 to a pulse width
  // between 1000 and 2000.
  // The suffix _us stands for microseconds.
  int pulse_width_us = map(input, 0, 1023, 1000, 2000);
  // Set the servo PWM pulse width to that value.
  servo.writeMicroseconds(pulse_width_us);
}