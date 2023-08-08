#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>

class MotorKalmanFilter 
{
  public:
    MotorKalmanFilter();
    void update(double dt, double enc_diff);
    double getVel();
      
  private:
    // kalman filter state
    BLA::Matrix<2> state = { 0.0, 0.0 }; // [ang_pos, ang_vel]
    BLA::Matrix<2, 2> state_cov = { 1.0, 0.0, 0.0, 1.0 };

    // fixed error covariances
    BLA::Matrix<2, 2> transition_error = { 0.01, 0.0, 0.0, 0.01 };
    BLA::Matrix<1, 1> measurement_error = { 0.1 };

    // transition and measurement models
    BLA::Matrix<2, 2> A = { 1.0, 0.0, 0.0, 1.0 };  // A(0, 1) should be updated on update step
    BLA::Matrix<1, 2> C = { 0.0, 0.0 };            // C(0, 1) should be updated on update step
    BLA::Matrix<2, 2> I = { 1.0, 0.0, 0.0, 1.0 }; 
};
