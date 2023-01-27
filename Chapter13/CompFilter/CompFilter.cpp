#include "CompFilter.h"

CompFilter::CompFilter(float coeff) {
  filter_coeff_ = coeff;
}

float CompFilter::getAngleFiltered() {
  return angle_filtered_;
}

float CompFilter::getAngleAccel() {
  return angle_accel_;
}

float CompFilter::getAngleGyro() {
  return angle_gyro_;
}

void CompFilter::setGyroOffset(float offset) {
  gyro_offset_ = offset;
}

void CompFilter::update(float rate_z /*in rad/s*/, float accel_x, float accel_y,
                        float sampling_interval /*us*/) {
  // Accelerometer update:
  angle_accel_ = atan2(accel_x, accel_y);
  if (!initialized_) {
    angle_gyro_ = angle_accel_;
    angle_filtered_ = angle_accel_;
    initialized_ = true;
  }
  // Gyro update:
  float gyro_delta = 0.000001 * ((gyro_offset_ - rate_z) * sampling_interval);
  angle_gyro_ += gyro_delta;
  // Tilt update with complementary filter.
  angle_filtered_ = filter_coeff_ * (angle_filtered_ + gyro_delta) +
                    (1.0 - filter_coeff_) * angle_accel_;
}
