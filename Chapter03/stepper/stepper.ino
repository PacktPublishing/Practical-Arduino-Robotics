// Include stepper motor library.
#include <Stepper.h>
// Define the pin numbers for phase A.
const int dir_pin_a = 12;
const int brake_pin_a = 9;
const int pwm_pin_a = 3;
// Define the pin numbers for phase B.
const int dir_pin_b = 13;
const int brake_pin_b = 8;
const int pwm_pin_b = 11;
// Analog input pins.
const int analog_pin_power = A5;
const int analog_pin_speed = A4;
// Most steppers have 200 steps per revolution.
const int stepsPerRev = 200;

// Instantiate a stepper motor object.
Stepper bipolarStepper(stepsPerRev, dir_pin_a, dir_pin_b);

void setup() {
  // Set control pins as outputs.
  pinMode(dir_pin_a, OUTPUT);
  pinMode(brake_pin_a, OUTPUT);
  pinMode(dir_pin_b, OUTPUT);
  pinMode(brake_pin_a, OUTPUT);
  // Disable brake functionality.
  digitalWrite(brake_pin_a, LOW);
  digitalWrite(brake_pin_b, LOW);
}

void loop() {
  // Map power potentiometer to a PWM output.
  int power_input = analogRead(analog_pin_power);
  int output_pwm = map(power_input, 0, 1023, 0, 255);
  // Map speed potentiometer to an rpm output.
  int speed_input = analogRead(analog_pin_speed);
  int rpm = map(speed_input, 0, 1023, 0, 100);
  // Drive both phases with the same PWM.
  analogWrite(pwm_pin_a, output_pwm);
  analogWrite(pwm_pin_b, output_pwm);
  // Set the motor speed.
  bipolarStepper.setSpeed(rpm);
  // Only call step() if rpm is greater than 0.
  if (rpm > 0) {
    bipolarStepper.step(1);
  }
}