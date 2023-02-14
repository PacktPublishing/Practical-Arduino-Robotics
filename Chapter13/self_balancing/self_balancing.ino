// Install via Library Manager:
#include <Adafruit_LSM6DS33.h>
// Custom library developed in chapter 06:
#include "Blinker.h"
// Custom libraries included in chapter 13:
#include "CompFilter.h"
#include "MotorDriver.h"
#include "QuadEncoder.h"

/*************** PIN ASSIGNMENTS ***************/
// Interrupt pins: 2, 3, 18, 19, 20, 21
// Timer 1 controls PWM pins 11, 12

// SPI pins. All but CS are assigned automatically.
// MOSI: 51
// MISO: 50
// SCK: 52
const int kSpiCsPin = 10;

// Power system pins and parameters.
const int kBatteryVoltagePin = A0;  // pin 54
const float kMinBatteryVoltage = 10.5;

namespace left {
const int kEncoderPinA = 20;  // Interrupt. Conflicts with I2C. Yellow.
const int kEncoderPinB = 21;  // Interrupt. Conflicts with I2C. White.
const int kMotorPwmPin = 11;  // Adjust PWM frequency for this pin.
const int kMotorDirPinA = 4;  // Simple digital output.
const int kMotorDirPinB = 5;  // Simple digital output.
MotorDriver motor(kMotorPwmPin, kMotorDirPinA, kMotorDirPinB);
QuadEncoder encoder(kEncoderPinA, kEncoderPinB);
// Global wrappers for the interrupt handlers.
void IsrA() {
  encoder.changeIsrA();
}
void IsrB() {
  encoder.changeIsrB();
}
}

namespace right {
const int kEncoderPinA = 18;  // Interrupt. Conflicts with Serial1. Yellow.
const int kEncoderPinB = 19;  // Interrupt. Conflicts with Serial1. White.
const int kMotorPwmPin = 12;  // Adjust PWM frequency for this pin.
const int kMotorDirPinA = 8;  // Simple digital output.
const int kMotorDirPinB = 9;  // Simple digital output.
MotorDriver motor(kMotorPwmPin, kMotorDirPinA, kMotorDirPinB);
QuadEncoder encoder(kEncoderPinA, kEncoderPinB);
// Global wrappers for the interrupt handlers.
void IsrA() {
  encoder.changeIsrA();
}
void IsrB() {
  encoder.changeIsrB();
}
}

// RC
const int kPwmCenter = 1500;

namespace steer {
volatile unsigned long rise_time_us;
volatile int pulse_width_us;
const int pwm_input_pin = 2;  // Receiver channel 1.
void risingISR();             // Declare this function here so we can point to it.
void fallingISR() {
  pulse_width_us = micros() - rise_time_us;
  attachInterrupt(digitalPinToInterrupt(pwm_input_pin), risingISR, RISING);
}
void risingISR() {
  rise_time_us = micros();
  attachInterrupt(digitalPinToInterrupt(pwm_input_pin), fallingISR, FALLING);
}
}

namespace throttle {
volatile unsigned long rise_time_us;
volatile int pulse_width_us;
const int pwm_input_pin = 3;  // Receiver channel 3.
void risingISR();             // Declare this function here so we can point to it.
void fallingISR() {
  pulse_width_us = micros() - rise_time_us;
  attachInterrupt(digitalPinToInterrupt(pwm_input_pin), risingISR, RISING);
}
void risingISR() {
  rise_time_us = micros();
  attachInterrupt(digitalPinToInterrupt(pwm_input_pin), fallingISR, FALLING);
}
}

/*************** SETTINGS ***************/
// Arduino settings.
// Timer/Counter1 prescaler setting for motor PWM.
const int kTimer1PrescalerSetting = 0b001;
const long kSerialBaudrate = 115200;

// IMU
const int kImuDataRate = LSM6DS_RATE_833_HZ;  // Faster than the main control rate.
const int kNumImuInitSamples = 100;
const int kNumImuCalibSamples = 100;
const float kCompFilterCoeff = 0.999;  // The higher the smoother.

// Controller gains.
float kp_balance = -8000;
float kd_balance = -100;

float kp_position = 0;
float kd_position = 0.15;

float kd_rotation = -100;

// RC input gains.
float kp_steer = -0.6;
float kp_throttle = 2.5;

// Other controller parameters.
const int kDeadbandPwm = 25;
const float kPitchOffset = 0.05;
const int kCutoffPitchDeg = 45;
/*************** CONTROL FLAGS ***************/
bool motors_enabled = false;
bool battery_ok = true;

/*************** SCHEDULING ***************/
namespace control_task {
const int kInterval = 2;  // ms
long last_time;
}

namespace print_task {
const int kInterval = 50;  // ms
long last_time;
}

Blinker myBlinker(13, 500);

/*************** IMU ***************/
Adafruit_LSM6DS33 imu;  // IMU.
// Containers for IMU data.
sensors_event_t accel, gyro, temp;
CompFilter pitchFilter(kCompFilterCoeff);  // Filter for pitch angle estimation.

/*************** HELPER FUNCTIONS ***************/
float rad2deg(float rad) {
  return 180.0 * rad / PI;
}

int deadbandCompensation(int input, int deadband_offset) {
  if (input > 0) {
    return input + deadband_offset;
  } else {
    return input - deadband_offset;
  }
}

void parseFloatParameter(float &parameter) {
  if (Serial.available()) {
    parameter = Serial.parseFloat();
    // Flush the input buffer.
    while (Serial.available()) {
      Serial.read();
    }
  }
}

float getBatteryVoltage() {
  const float R_sense = 4.7;                             // kOhm
  const float R_series = 10.0;                           // kOhm
  float U_sense = 5.0 * (float)analogRead(A0) / 1023.0;  // V
  // Compute battery voltage in Volts.
  return U_sense * (R_series + R_sense) / R_sense;       // V
}

void setup() {
  // Initialize serial interfaces first so that we can print debug messages.
  Serial.begin(kSerialBaudrate);  // Serial interface to USB.

  // Initialize Blinker.
  myBlinker.begin();

  // Attach encoder interrupt handlers to interrupt sources.
  attachInterrupt(digitalPinToInterrupt(left::encoder.getPinA()), left::IsrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left::encoder.getPinB()), left::IsrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right::encoder.getPinA()), right::IsrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right::encoder.getPinB()), right::IsrB, CHANGE);

  // Attach RC input interrupt handlers.
  attachInterrupt(digitalPinToInterrupt(steer::pwm_input_pin), steer::risingISR, RISING);
  attachInterrupt(digitalPinToInterrupt(throttle::pwm_input_pin), throttle::risingISR, RISING);

  // Change timer 1 prescaler. Timer 1 generates the PWM on pins 11 and 12
  // on the Arduino Mega2560.
  // The three least significant bits in the Timer Counter Control Register B
  // for timer 1 (TCCR1B) control the prescaler. Setting them to 0b001 sets the prescaler
  // to 1 and results in a PWM frequency of 31.37255 kHz. This is above the audible range
  // and should result in low noise and good DC motor performance.
  TCCR1B = TCCR1B & 0b11111000 | kTimer1PrescalerSetting;

  // Initialize motor drivers.
  left::motor.begin();
  right::motor.begin();

  // Initialize encoders.
  left::encoder.begin();
  right::encoder.begin();

  // Initialize IMU.
  imu.begin_SPI(kSpiCsPin);
  imu.setAccelDataRate(kImuDataRate);
  imu.setGyroDataRate(kImuDataRate);
  // Collect a few dummy readings that won't be used to make sure subsequent readings are valid.
  for (int i = 0; i < kNumImuInitSamples; i++) {
    imu.getEvent(&accel, &gyro, &temp);
  }
  // Gyro calibration data.
  float gyro_x_offset = 0.0;
  for (int i = 0; i < kNumImuCalibSamples; i++) {
    imu.getEvent(&accel, &gyro, &temp);
    gyro_x_offset += gyro.gyro.x;
  }
  gyro_x_offset /= (float)kNumImuCalibSamples;
  pitchFilter.setGyroOffset(gyro_x_offset);

  // Initialize task scheduling.
  control_task::last_time = millis();
  print_task::last_time = millis();
}

void loop() {
  // Status blinking.
  myBlinker.blink();
  if (getBatteryVoltage() < kMinBatteryVoltage && battery_ok) {
    battery_ok = false;
    // Signal low battery with high frequency blinking.
    myBlinker.set_blink_interval(50);
  }

  // High frequency encoder updates
  left::encoder.update();
  right::encoder.update();

  // Read parameter from Serial port for interactive tuning.
  // Can be used for any parameter of type float.
  parseFloatParameter(kp_balance);

  // Task that samples the IMU, performs a pitch filter update, and runs all controllers.
  if (millis() > control_task::last_time + control_task::kInterval) {
    control_task::last_time += control_task::kInterval;
    // Get IMU data.
    imu.getEvent(&accel, &gyro, &temp);
    // Perform pitch filter update.
    pitchFilter.update(gyro.gyro.x, accel.acceleration.z, accel.acceleration.y, control_task::kInterval * 1000);
    // Enable motors when the robot is upright.
    if ((abs(kPitchOffset - pitchFilter.getAngleFiltered()) < 0.01) && (!motors_enabled)) {
      motors_enabled = true;
    }
    if (abs(rad2deg(pitchFilter.getAngleFiltered())) > kCutoffPitchDeg) {
      motors_enabled = false;
    }

    // All controllers output motor PWM values that are accumulated.
    // Balance controller.
    int pwm = 0;
    // diff_pwm gets applied to the motors with different sign.
    int diff_pwm = 0;
    pwm += kp_balance * (kPitchOffset - pitchFilter.getAngleFiltered());  // P-control.
    pwm += kd_balance * gyro.gyro.x;                                      // D-control.

    // Position controller
    // Linear position and velocity are the averages of the two encoders. Does not change / is 0 when the robot turns in place.
    float linear_position = (float)(right::encoder.getPosition() + left::encoder.getPosition()) / 2.0;
    float linear_velocity = (float)(right::encoder.getVelocity() + left::encoder.getVelocity()) / 2.0;
    pwm += kp_position * linear_position;
    pwm += kd_position * linear_velocity;
    pwm += kp_throttle * (float)(throttle::pulse_width_us - kPwmCenter);

    diff_pwm += kd_rotation * gyro.gyro.y;
    diff_pwm += kp_steer * (float)(steer::pulse_width_us - kPwmCenter);

    // Set PWM or turn off motors if motors are disabled.
    if (!motors_enabled) {
      left::motor.setPwm(0);
      right::motor.setPwm(0);
    } else {
      left::motor.setPwm(deadbandCompensation(pwm + diff_pwm, kDeadbandPwm));
      right::motor.setPwm(deadbandCompensation(pwm - diff_pwm, kDeadbandPwm));
    }
  }

  // High rate debug output.
  if (millis() > print_task::last_time + print_task::kInterval) {
    print_task::last_time += print_task::kInterval;
    Serial.print(rad2deg(pitchFilter.getAngleFiltered()));
  }
}
