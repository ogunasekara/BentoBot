#include "Arduino.h"
#include "MotorLowPassFilter.h"

#define TICKS_PER_REV 680.0
#define WHEEL_DIAMETER 0.068

MotorLowPassFilter::MotorLowPassFilter(double alpha): alpha { alpha } {}

void MotorLowPassFilter::update(double dt, double enc_diff) {
  if (dt <= 0) { return; }

  double ang_vel_raw = (enc_diff / dt) * (2 * PI / TICKS_PER_REV);
  ang_vel += alpha * (ang_vel_raw - ang_vel);
}

// returns angular velocity of the wheel (rad/s)
double MotorLowPassFilter::getAngVel() {
  return ang_vel;
}

// returns linear velocity of the wheel (m/s), same as MotorKalmanFilter::getVel
double MotorLowPassFilter::getVel() {
  return ang_vel * (WHEEL_DIAMETER / 2.0);
}
