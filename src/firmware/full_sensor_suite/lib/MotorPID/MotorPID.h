#pragma once

#include "Arduino.h"
#include "MotorModel.h"

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
      MotorModel model,
      double wheel_radius,
      double max_accel = 1.5
    );
    void begin();
    void update(double dt, double cur_vel);
    void setVel(double vel);
    void driveMotor(int motor_cmd);
      
  private:
    unsigned int MOTOR_IN1;
    unsigned int MOTOR_IN2;
    unsigned int MOTOR_PWM;

    double PID_P;
    double PID_I;
    double PID_D;
    MotorModel model; // PWM feed forward model
    double wheel_radius; // m, converts wheel velocity to angular velocity for the model
    double max_accel; // m/s^2, limit on how fast vel_target ramps toward vel_goal

    double integral;
    double derivative;
    double vel_goal = 0.0; // requested velocity
    double vel_target = 0.0; // ramped velocity the controller tracks
    double feed_forward;
    double diff;

    int motor_cmd;
};
