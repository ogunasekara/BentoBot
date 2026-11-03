#pragma once

#include "Arduino.h"

class MotorLowPassFilter
{
  public:
    MotorLowPassFilter(double alpha = 0.2);
    void update(double dt, double enc_diff);
    double getAngVel();
    double getVel();

  private:
    double alpha; // smoothing factor, higher = less smoothing
    double ang_vel = 0.0; // rad/s
};
