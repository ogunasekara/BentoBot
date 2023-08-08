#include "Arduino.h"
#include "MotorKalmanFilter.h"
#include <BasicLinearAlgebra.h>

#define TICKS_PER_REV 680.0
#define WHEEL_DIAMETER 0.068

MotorKalmanFilter::MotorKalmanFilter() {}

void MotorKalmanFilter::update(double dt, double enc_diff) {
  A(0, 1) = dt;
  C(0, 1) = dt * (TICKS_PER_REV / (2 * PI));
  BLA::Matrix<1, 1> z = { enc_diff };

  BLA::Matrix<2> state_tmp = A * state;
  BLA::Matrix<2, 2> state_cov_tmp = A * state_cov * ~A + transition_error;

  BLA::Matrix<1, 1> S_inv = C * state_cov_tmp * ~C + measurement_error;
  BLA::Invert(S_inv);

  BLA::Matrix<2, 1> K = (state_cov_tmp * (~C)) * S_inv;

  state = state_tmp + K * (z - (C * state_tmp));
  state_cov = (I - K * C) * state_cov_tmp;
}

// returns linear velocity of the wheel
double MotorKalmanFilter::getVel() {
  return state(1) * (WHEEL_DIAMETER / 2.0);
}
