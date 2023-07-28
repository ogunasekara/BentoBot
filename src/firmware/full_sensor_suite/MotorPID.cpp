#include "Arduino.h"
#include "MotorPID.h"

#define PWM_MIN 10
#define PWM_MAX 255
#define INTEGRAL_DECAY 0.01


MotorPID::MotorPID(
  unsigned int MOTOR_IN1,
  unsigned int MOTOR_IN2,
  unsigned int MOTOR_PWM,
  double PID_P,
  double PID_I,
  double PID_D,
  double FF_A,
  double FF_B):
  MOTOR_IN1 { MOTOR_IN1 },
  MOTOR_IN2 { MOTOR_IN2 },
  MOTOR_PWM { MOTOR_PWM },
  PID_P { PID_P },
  PID_I { PID_I },
  PID_D { PID_D },
  FF_A { FF_A },
  FF_B { FF_B }
{}

void MotorPID::begin() {
  // setup motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // ensure motors are not moving
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

void MotorPID::update(double dt, double cur_vel) {
  if (abs(vel_target) > 0) {
    diff = (vel_target - cur_vel);

    derivative = diff / dt;
    integral += diff * dt;  // TODO: incorporate decay into this term

    feed_forward = FF_A * exp(FF_B * abs(vel_target)); // calculated experimentally and fitted to exponential
    feed_forward = (vel_target / abs(vel_target)) * feed_forward; // match sign with vel_target

    motor_cmd = ((feed_forward) + (PID_P * diff) + (PID_I * integral) + (PID_D * derivative));
    motor_cmd = (motor_cmd / abs(motor_cmd)) * min(PWM_MAX, abs(motor_cmd));  // upper bound
  } else {
    motor_cmd = 0;
  }

    driveMotor(motor_cmd);
}

void MotorPID::setVel(double vel) {
  vel_target = vel;
}

void MotorPID::driveMotor(int motor_cmd) {
  // stop motor if motor cmd too low
  if (abs(motor_cmd) <= PWM_MIN) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (motor_cmd > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, abs(motor_cmd));
  } else if (motor_cmd < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, abs(motor_cmd));
  }
}
