#include "Arduino.h"

#ifndef _COMP_FILTER_H
#define _COMP_FILTER_H

class CompFilter {
public:
  // Constructor.
  CompFilter(float coeff);
  // Get the angle estimate produced by the complementary filter.
  float getAngleFiltered();
  // Get the angle estimate from the accelerometer.
  float getAngleAccel();
  // Get the angle estimate from the gyroscope.
  float getAngleGyro();
  // Set the fixed gyroscope offset.
  void setGyroOffset(float offset);

  // Given the angular rate around one axis and accelerations along the other two
  // axes as well as a sampling interval, perform an incremental tilt estimation
  // step.
  void update(float rate_z /*in rad/s*/, float accel_x, float accel_y,
              float sampling_interval /*us*/);

private:
  float angle_filtered_ = 0;
  float angle_gyro_ = 0;
  float angle_accel_ = 0;
  float filter_coeff_ = 0.9;
  float gyro_offset_ = 0;
  bool initialized_ = false;
};

#endif  // _COMP_FILTER_H
