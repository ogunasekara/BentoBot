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
      double wheel_radius
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

    double integral;
    double derivative;
    double vel_target;
    double feed_forward;
    double diff;

    int motor_cmd;
};
