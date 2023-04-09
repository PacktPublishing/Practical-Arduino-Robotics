// Pixy2 libarary that you can downloard here: https://pixycam.com/downloads-pixy2/
#include <Pixy2.h>
// Library developed in chapter 06:
#include "Blinker.h"

// Instantiate a Pixy2 and a Blinker object.
Pixy2 pixy;
Blinker status_blinker(13, 500);

// This will be compiled if you use an Arduino Uno.
#if defined(__AVR_ATmega328P__)
#include <SoftwareSerial.h>
// Pin assignments for Arduino Uno.
// Using high-frequency PWM pins for motor control.
// Camera SPI MOSI: 11
// Camera SPI MISO: 12
// Camera SPI SCK: 13

const int ble_RX_pin = 2;
const int ble_TX_pin = 3;

const int left_motor_pwm_pin = 5;  // 980 Hz PWM.
const int left_motor_dir_pin_A = 4;
const int left_motor_dir_pin_B = 7;

const int right_motor_pwm_pin = 6;  // 980 Hz PWM.
const int right_motor_dir_pin_A = 9;
const int right_motor_dir_pin_B = 8;

// Use SoftwareSerial for Bluetooth.
SoftwareSerial ble(ble_RX_pin, ble_TX_pin);

// This will be compiled if you use an Arduino Mega2560.
#elif defined(__AVR_ATmega2560__)
// Pin assignments for Arduino Mega.
// Taking into account the self-balancing robot.
// Camera SPI MOSI: 51
// Camera SPI MISO: 50
// Camera SPI SCK: 52

const int left_motor_pwm_pin = 11;
const int left_motor_dir_pin_A = 5;
const int left_motor_dir_pin_B = 4;

const int right_motor_pwm_pin = 12;
const int right_motor_dir_pin_A = 9;
const int right_motor_dir_pin_B = 8;

// Use HardwareSerial port 2 for Bluetooth.
HardwareSerial& ble = Serial2;  // ble is a reference to Serial2.

#else
#error Invalid board. Use Arduino Uno or Mega2560.
#endif  // #if defined(UNO)

// Controller parameters.
// The effect of pressing an arrow button in manual mode.
const int button_increment = 50;
// Upper limit of the motor PWM (in case the battery voltage is higher
// than the motors' nominal voltage).
const int max_motor_pwm = 200;
// PWM commanded to the motors when going straight.
const int line_follow_pwm_offset = 45;
// Proportional gain of the line following controller.
float kp_line = 3.5;

// Power parameters.
const float min_batt_voltage = 10.0;

// Robot Modes.
enum Mode {
  MANUAL_CONTROL,
  FOLLOW_LINE
};

Mode mode = MANUAL_CONTROL;

// Motor class.
class Motor {
public:
  // Constructor.
  Motor(int pwm_pin, int dir_pin_A, int dir_pin_B) {
    pwm_pin_ = pwm_pin;
    dir_pin_A_ = dir_pin_A;
    dir_pin_B_ = dir_pin_B;
  }

  void begin(int max_pwm) {
    pinMode(dir_pin_A_, OUTPUT);
    pinMode(dir_pin_B_, OUTPUT);
    digitalWrite(dir_pin_A_, LOW);
    digitalWrite(dir_pin_B_, LOW);
    analogWrite(pwm_pin_, 0);
    max_pwm_ = max_pwm;
  }

  void setPwm(int pwm) {
    // Constrain pwm to protect the motor.
    pwm_ = constrain(pwm, -max_pwm_, max_pwm_);
    // Set direction according to sign of pwm parameter.
    bool dir = pwm_ > 0;
    digitalWrite(dir_pin_A_, dir);
    digitalWrite(dir_pin_B_, !dir);
    int command_pwm = abs(pwm_);
    analogWrite(pwm_pin_, command_pwm);
  }

  void adjustPwm(int val) {
    setPwm(pwm_ + val);
  }

  int getPwm() {
    return pwm_;
  }

private:
  int pwm_pin_ = 0;
  int dir_pin_A_ = 0;
  int dir_pin_B_ = 0;
  int pwm_ = 0;
  int max_pwm_ = 255;
};

Motor left_motor(left_motor_pwm_pin, left_motor_dir_pin_A, left_motor_dir_pin_B);
Motor right_motor(right_motor_pwm_pin, right_motor_dir_pin_A, right_motor_dir_pin_B);

void setup() {
  // Serial interface to USB.
  Serial.begin(115200);
  // Serial interface to Bluetooth.
  ble.begin(9600);
  // Initialize blinker.
  status_blinker.begin();
  // Initialize motor driver pins.
  left_motor.begin(max_motor_pwm);
  right_motor.begin(max_motor_pwm);
  // Camera.
  Serial.println("Starting camera...");
  pixy.init();
  Serial.println(pixy.changeProg("line_tracking"));
}

void loop() {
  parseBle();
  if (mode == FOLLOW_LINE) {
    followLine();
  }
  monitorBattery();
  status_blinker.blink();
}

void followLine() {
  // Get line detection results from Pixy.
  int res = pixy.line.getMainFeatures();
  if (res <= 0) {
    // No line detected. Stop motors and print error message.
    left_motor.setPwm(0);
    right_motor.setPwm(0);
    ble.println("No line found.");
    return;
  }
  if (res & LINE_VECTOR) {  // Line detected.
    // Lateral pixel coordinate of the far end of the line vector.
    long x_far = (long)pixy.line.vectors->m_x1;
    // Lateral pixel coordinate of the near end of the line vector.
    long x_near = (long)pixy.line.vectors->m_x0;
    // Average line deviation from frame center.
    long error = (x_far + x_near - pixy.frameWidth) / 2;
    // Line following controller.
    left_motor.setPwm(line_follow_pwm_offset + kp_line * error);
    right_motor.setPwm(line_follow_pwm_offset - kp_line * error);
  }
}

void parseBle() {
  int right_pwm_increment_factor = 0;
  int left_pwm_increment_factor = 0;
  bool print_enabled = true;
  const int ble_datalen = 10;
  char databuf[ble_datalen];
  if (ble.available() == ble_datalen) {
    for (int i = 0; i < ble_datalen; i++) {
      databuf[i] = ble.read();
    }
    // Data is ready to parse.
    if (databuf[2] == '1') {
      Serial.println(1);
      ble.println("Manual Control");
      // Change mode.
      mode = MANUAL_CONTROL;
      // Turn off camera lamp.
      pixy.setLamp(0, 0);
      // Stop both motors.
      left_motor.setPwm(0);
      right_motor.setPwm(0);
      // Prevent print in this iteration.
      print_enabled = false;
    } else if (databuf[2] == '2') {
      Serial.println(2);
      ble.println("Line Following");
      // Change mode.
      mode = FOLLOW_LINE;
      // Turn on camera lamps.
      pixy.setLamp(100, 100);
    } else if (databuf[2] == '5') {
      Serial.println("UP");
      // Both motors faster.
      left_pwm_increment_factor = 1;
      right_pwm_increment_factor = 1;
    } else if (databuf[2] == '6') {
      Serial.println("DOWN");
      // Both motors slower.
      left_pwm_increment_factor = -1;
      right_pwm_increment_factor = -1;
    } else if (databuf[2] == '7') {
      Serial.println("LEFT");
      // Left motor slower, right motor faster.
      left_pwm_increment_factor = -1;
      right_pwm_increment_factor = 1;
    } else if (databuf[2] == '8') {
      Serial.println("RIGHT");
      // Left motor faster, right motor slower.
      left_pwm_increment_factor = 1;
      right_pwm_increment_factor = -1;
    }
    if (print_enabled && (mode == MANUAL_CONTROL)) {
      // If under manual control, adjust the motor speed
      // and print out the updated motor speeds over Bluetooth.
      left_motor.adjustPwm(left_pwm_increment_factor * button_increment);
      right_motor.adjustPwm(right_pwm_increment_factor * button_increment);
      ble.print(left_motor.getPwm());
      ble.print('\t');
      ble.println(right_motor.getPwm());
    }
  }
}

void monitorBattery() {
  if (getBatteryVoltage() < min_batt_voltage) {
    // Put into manual mode, stop motors and turn off lamp.
    mode = MANUAL_CONTROL;
    left_motor.setPwm(0);
    right_motor.setPwm(0);
    pixy.setLamp(0, 0);
    ble.println("Error: Battery voltage low.");
  }
}

float getBatteryVoltage() {
  const float R_sense = 4.7;                             // kOhm
  const float R_series = 10.0;                           // kOhm
  float U_sense = 5.0 * (float)analogRead(A0) / 1023.0;  // V
  // Compute battery voltage in Volts.
  return U_sense * (R_series + R_sense) / R_sense;  // V
}
