#pragma once

#include "Arduino.h"

class MotorPID 
{
  public:
    MotorPID(
      unsigned int MOTOR_IN1,
      unsigned int MOTOR_IN2,
      unsigned int MOTOR_PWM,
      double PID_P,
      double PID_I,
      double PID_D,
      double FF_A,
      double FF_B
    );
    void begin();
    void update(double dt, double cur_vel);
    void setVel(double vel);
    void setMotorCmd(int cmd);
      
  private:
    void driveMotor(int motor_cmd);

    unsigned int MOTOR_IN1;
    unsigned int MOTOR_IN2;
    unsigned int MOTOR_PWM;

    double PID_P;
    double PID_I;
    double PID_D;
    double FF_A; // feed forward multiplier
    double FF_B; // feed forward exponential multiplier

    double integral;
    double derivative;
    double vel_target;
    double feed_forward;
    double diff;

    int motor_cmd;
};
