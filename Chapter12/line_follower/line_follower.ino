#include <Pixy2.h>
#include "Blinker.h"

Pixy2 pixy;
Blinker status_blinker(13, 200);

#if defined(__AVR_ATmega328P__) // Arduino Uno.
#include <SoftwareSerial.h>
// Pin assignments for Arduino Uno.
// Camera SPI MOSI: 11
// Camera SPI MISO: 12
// Camera SPI SCK: 13

const int ble_RX_pin = 2;
const int ble_TX_pin = 3;

const int left_motor_pwm_pin = 5; // 980 Hz PWM.
const int left_motor_dir_pin_A = 4;
const int left_motor_dir_pin_B = 7;

const int right_motor_pwm_pin = 6; // 980 Hz PWM.
const int right_motor_dir_pin_A = 9;
const int right_motor_dir_pin_B = 8;

// Use SoftwareSerial for Bluetooth.
SoftwareSerial ble(ble_RX_pin, ble_TX_pin);

#elif defined(__AVR_ATmega2560__) // Arduino Mega2560.
// Pin assignments for Arduino Mega.
// Camera SPI MOSI: 51
// Camera SPI MISO: 50
// Camera SPI SCK: 52

const int left_motor_pwm_pin = 4; // 980 Hz PWM.
const int left_motor_dir_pin_A = 6;
const int left_motor_dir_pin_B = 5;

const int right_motor_pwm_pin = 13; // 980 Hz PWM.
const int right_motor_dir_pin_A = 9;
const int right_motor_dir_pin_B = 8;

// Use HardwareSerial port 1 for Bluetooth.
HardwareSerial& ble = Serial2;  // ble is a reference to Serial1. 

#else
#error Invalid board. Use Arduino Uno or Mega2560.
#endif // #if defined(UNO)

// Controller parameters.
const int button_increment = 50;
const int max_pwm = 200;
const int line_follow_pwm_offset = 55;
float kp_line = 4.0;

// Power parameters.
const float min_batt_voltage = 10.0;

// Robot Modes.
enum Mode {
  manual_control,
  follow_line
};

Mode mode = manual_control;

// Motor class.
class Motor {
  public:
    // Constructor.
    Motor(int pwm_pin, int dir_pin_A, int dir_pin_B) {
      pwm_pin_ = pwm_pin;
      dir_pin_A_ = dir_pin_A;
      dir_pin_B_ = dir_pin_B;
      max_pwm_ = max_pwm;
    }

    void Initialize(int max_pwm) {
      pinMode(dir_pin_A_, OUTPUT);
      pinMode(dir_pin_B_, OUTPUT);
      digitalWrite(dir_pin_A_, LOW);
      digitalWrite(dir_pin_B_, LOW);
      analogWrite(pwm_pin_, 0);
      max_pwm_ = max_pwm;
    }

    void SetPwm(int pwm) {
      // Constrain pwm to protect the motor.
      pwm_ = constrain(pwm, -max_pwm_, max_pwm_);
      // Set directio according to sign of pwm parameter.
      bool dir = pwm_ > 0;
      digitalWrite(dir_pin_A_, dir);
      digitalWrite(dir_pin_B_, !dir);
      int command_pwm = abs(pwm);
      analogWrite(pwm_pin_, command_pwm);
    }

    void AdjustPwm(int val) {
      SetPwm(pwm_ + val);
    }

    int GetPwm() {
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
  left_motor.Initialize(max_pwm);
  right_motor.Initialize(max_pwm);
  // Camera.
  Serial.println("Starting camera...");
  pixy.init();
  Serial.println(pixy.changeProg("line_tracking"));
}

void loop() {
  ParseBle();
  
  if (mode == follow_line) {
    FollowLine();
  }
  
  MonitorBattery();
  status_blinker.blink();
}

void FollowLine() {
  // Get line detection results from Pixy.
  int res = pixy.line.getMainFeatures();
  if (res <= 0) { // No line detected.
    left_motor.SetPwm(0);
    right_motor.SetPwm(0);
    Serial2.println("No line found.");
    return;
  }
  if (res & LINE_VECTOR) { // Line detected.
    // Lateral pixel coordinate of the far end of the line vector.
    long x_far = (long)pixy.line.vectors->m_x1;
    // Lateral pixel coordinate of the near end of the line vector.
    long x_near = (long)pixy.line.vectors->m_x0;
    // Average line deviation from frame center.
    long error = (x_far + x_near - pixy.frameWidth) / 2;
    // Line following controller.
    left_motor.SetPwm(line_follow_pwm_offset + kp_line * error);
    right_motor.SetPwm(line_follow_pwm_offset - kp_line * error);
  }
}

void ParseBle() {
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
      mode = manual_control; // Change mode.
      pixy.setLamp(0, 0); // Turn off camera lamp.
      left_motor.SetPwm(0); // Stop left motor.
      right_motor.SetPwm(0); // Stop right motor.
      print_enabled = false; // Prevent print in this iteration.
    }
    else if (databuf[2] == '2') {
      Serial.println(2);
      ble.println("Line Following");
      mode = follow_line; // Change mode.
      pixy.setLamp(255, 255); // Turn on camera lamps.
    }
    else if (databuf[2] == '5') {
      Serial.println("UP");
      // Both motors faster.
      left_pwm_increment_factor = 1;
      right_pwm_increment_factor = 1;
    }
    else if (databuf[2] == '6') {
      Serial.println("DOWN");
      // Both motors slower.
      left_pwm_increment_factor = -1;
      right_pwm_increment_factor = -1;
    }
    else if (databuf[2] == '7') {
      Serial.println("LEFT");
      // Left motor slower, right motor faster.
      left_pwm_increment_factor = -1;
      right_pwm_increment_factor = 1;
    }
    else if (databuf[2] == '8') {
      Serial.println("RIGHT");
      // Left motor faster, right motor slower.
      left_pwm_increment_factor = 1;
      right_pwm_increment_factor = -1;
    }
    if (print_enabled && (mode == manual_control)) {
      // If under manual control, adjust the motor speed
      // and print out the updated motor speeds over Bluetooth.
      left_motor.AdjustPwm(left_pwm_increment_factor * button_increment);
      right_motor.AdjustPwm(right_pwm_increment_factor * button_increment);
      ble.print(left_motor.GetPwm());
      ble.print('\t');
      ble.println(right_motor.GetPwm());
    }
  }
}

void MonitorBattery() {
  if (GetBatteryVoltage() < min_batt_voltage) {
    // Put into manual mode, stop motors and turn off lamp.
    mode = manual_control;
    left_motor.SetPwm(0);
    right_motor.SetPwm(0);
    pixy.setLamp(0, 0);
    ble.println("Error: Battery voltage low.");
  }
}

float GetBatteryVoltage() {
  float U_sense = 5.0 * (float)analogRead(A0) / 1023.0; // V
  float R_series = 10.0; // kOhm
  float R_sense = 3.3; // kOhm
  return U_sense * (R_series + R_sense) / R_sense; // V
}
