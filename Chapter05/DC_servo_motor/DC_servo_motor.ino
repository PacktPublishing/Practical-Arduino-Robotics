// Variables for the print task.
unsigned long last_print_time = 0; 
const int print_interval = 20; 

// Variables of the controller task.
// Controller input.
long target_position = 0;
// Controller output.
float controller_output = 0;
// Output of the I control term.
float i_term = 0;
// Last error for computing the I and D term.
int last_error = 0;
// Variables for time keeping.
unsigned long last_control_time = 0;
const int control_interval = 10;

// Pins for motor control.
const int dir_pin_0 = 8;
const int dir_pin_1 = 9;
const int pwm_pin = 5;

// Use Arduino Uno interrupt pins.
// Pins 2 and 3 are our only options.
const int pin_A = 2;
const int pin_B = 3;

// Variables that are changed inside ISRs should always
// be volatile.
// Variables input pin states.
volatile bool state_a = LOW;
volatile bool state_b = LOW;
// Encoder position.
volatile long encoder_position = 0;


void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pin_A), risingIsrA, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_B), risingIsrB, RISING);
  pinMode(dir_pin_0, OUTPUT);
  pinMode(dir_pin_1, OUTPUT);
}

void loop() {
  if(millis() - last_control_time >= control_interval) {
    last_control_time += control_interval; 
    target_position = analogRead(A0);
    motorPidController();
  }
  if (millis() - last_print_time >= print_interval) { 
    last_print_time += print_interval; 
    // Print controller input, output, and encoder position.
    Serial.print("Target_position:");
    Serial.print(target_position); 
    Serial.print('\t'); 
    Serial.print("Actual_position:");
    Serial.print(encoder_position); 
    Serial.print('\t'); 
    Serial.print("Controller_output:");
    Serial.println(controller_output); 
  } 
}

void motorPidController() {
  // Controller gains.
  float kp = 1.0;
  float ki = 0.0;
  float kd = 60.0;
  // Compute the error (in encoder counts).
  int error = target_position - encoder_position;
  // Compute the P term.
  float p_term = kp * error;
  // Compute and constrain I term.
  i_term += ki * (error + last_error) / 2 * control_interval;
  i_term = constrain(i_term, -255, 255);
  // Compute D term.
  float d_term = kd * (error - last_error) / control_interval;
  // Compute final output.
  controller_output = p_term + i_term + d_term;
  // Update last_error for the next iteration.
  last_error = error;
  setMotorPwm(controller_output);
}

void setMotorPwm(int motor_pwm) {
  // Control motor direction pins based on the sign
  // of the motor_pwm parameter.
  if (motor_pwm > 0) {
    // Forward.
    digitalWrite(dir_pin_0, HIGH);
    digitalWrite(dir_pin_1, LOW);
  } else {
    // Backwards.
    digitalWrite(dir_pin_0, LOW);
    digitalWrite(dir_pin_1, HIGH);
    // Invert motor_pwm to make it positive.
    motor_pwm = -motor_pwm;
  }
  // Constrain motor_pwm between 0 and 255 to make it
  // compatible with analogWrite().
  motor_pwm = constrain(motor_pwm, 0, 255);
  analogWrite(pwm_pin, motor_pwm);
}

// Encoder logic table:
// ---------------------------------------
// Event     | Other pin state | direction
// ---------------------------------------
// rising A         HIGH            -
//                  LOW             +
// falling A        HIGH            +
//                  LOW             -
// rising B         HIGH            +
//                  LOW             -
// falling B        HIGH            -
//                  LOW             +

void risingIsrA() {
  state_a = HIGH;
  attachInterrupt(digitalPinToInterrupt(pin_A), fallingIsrA, FALLING);
  switch (state_b) {
    case LOW:
      // Encoder moved forward.
      encoder_position++;
      break;
    case HIGH:
      // Encoder moved backwards.
      encoder_position--;
      break;
  }
}

void risingIsrB() {
  state_b = HIGH;
  attachInterrupt(digitalPinToInterrupt(pin_B), fallingIsrB, FALLING);
  switch (state_a) {
    case LOW:
      // Encoder moved backwards.
      encoder_position--;
      break;
    case HIGH:
      // Encoder moved forward.
      encoder_position++;
      break;
  }
}

void fallingIsrA() {
  state_a = LOW;
  attachInterrupt(digitalPinToInterrupt(pin_A), risingIsrA, RISING);
  switch (state_b) {
    case LOW:
      // Encoder moved backwards.
      encoder_position--;
      break;
    case HIGH:
      // Encoder moved forward.
      encoder_position++;
      break;
  }
}

void fallingIsrB() {
  state_b = LOW;
  attachInterrupt(digitalPinToInterrupt(pin_B), risingIsrB, RISING);
  switch (state_a) {
    case LOW:
      // Encoder moved forward.
      encoder_position++;
      break;
    case HIGH:
      // Encoder moved backwards.
      encoder_position--;
      break;
  }
}