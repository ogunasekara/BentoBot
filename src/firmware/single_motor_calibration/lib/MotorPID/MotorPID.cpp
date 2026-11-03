#include "Arduino.h"
#include "MotorPID.h"

#define PWM_MAX 255
#define PWM_FREQUENCY 20000
#define INTEGRAL_DECAY 0.01


MotorPID::MotorPID(
  unsigned int MOTOR_IN1,
  unsigned int MOTOR_IN2,
  unsigned int MOTOR_PWM,
  double PID_P,
  double PID_I,
  double PID_D,
  MotorModel model,
  double wheel_radius,
  double max_accel):
  MOTOR_IN1 { MOTOR_IN1 },
  MOTOR_IN2 { MOTOR_IN2 },
  MOTOR_PWM { MOTOR_PWM },
  PID_P { PID_P },
  PID_I { PID_I },
  PID_D { PID_D },
  model { model },
  wheel_radius { wheel_radius },
  max_accel { max_accel }
{}

void MotorPID::begin() {
  // setup motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  analogWriteFrequency(MOTOR_PWM, PWM_FREQUENCY);
  
  // ensure motors are not moving
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

void MotorPID::update(double dt, double cur_vel) {
  // ramp the target toward the requested velocity
  double prev_target = vel_target;
  double max_step = max_accel * dt;
  if (vel_goal > vel_target + max_step) {
    vel_target += max_step;
  } else if (vel_goal < vel_target - max_step) {
    vel_target -= max_step;
  } else {
    vel_target = vel_goal;
  }

  // drop accumulated error on stop and on direction change
  if (prev_target * vel_target <= 0) {
    integral = 0;
  }

  if (abs(vel_target) > 0) {
    diff = (vel_target - cur_vel);
    derivative = diff / dt;

    if (abs(diff) > 0.1) {
      integral = 0;
    } else {
      integral += diff * dt;
    }

    feed_forward = model.getPWMFeedForward(vel_target / wheel_radius); // signed PWM from motor model

    double pwm = feed_forward + (PID_P * diff) + (PID_I * integral) + (PID_D * derivative);

    if (abs(pwm) < model.getCutoff()) {
      motor_cmd = 0;  // below the motor's deadband
    } else {
      motor_cmd = (pwm > 0 ? 1 : -1) * min((double)PWM_MAX, abs(pwm));  // upper bound
    }
  } else {
    motor_cmd = 0;
  }

    driveMotor(motor_cmd);
}

void MotorPID::setVel(double vel) {
  vel_goal = vel;
}

void MotorPID::driveMotor(int motor_cmd) {
  // stop motor if motor cmd below the model cutoff
  if (abs(motor_cmd) < model.getCutoff()) {
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
