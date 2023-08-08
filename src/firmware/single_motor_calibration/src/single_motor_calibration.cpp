#include <Arduino.h>
#include <MotorKalmanFilter.h>
#include <MotorPID.h>

/*--- MOTOR ENCODERS ---*/

// left motor
// #define MOTOR1_ENCA 11
// #define MOTOR1_ENCB 12
// #define MOTOR1_IN1 6
// #define MOTOR1_IN2 5
// #define MOTOR1_PWM 7

// right motor
#define MOTOR1_ENCA 10
#define MOTOR1_ENCB 9
#define MOTOR1_IN1 4
#define MOTOR1_IN2 3
#define MOTOR1_PWM 2

#define TICKS_PER_REV 680.0 // found through experimentation
#define WHEEL_DIAMETER 0.068
#define WHEEL_BASE_LENGTH 0.16

volatile long motor1_enc_count = 0;

void handleMotor1EncAChange() {
  int encA = digitalRead(MOTOR1_ENCA);
  int encB = digitalRead(MOTOR1_ENCB);

  if (encA == HIGH) {
    if (encB == HIGH) {
      motor1_enc_count += 1;
    } else {
      motor1_enc_count -= 1;
    }
  } else {
    if (encB == HIGH) {
      motor1_enc_count -= 1;
    } else {
      motor1_enc_count += 1;
    }
  }
}

void handleMotor1EncBChange() {
  int encA = digitalRead(MOTOR1_ENCA);
  int encB = digitalRead(MOTOR1_ENCB);

  if (encB == HIGH) {
    if (encA == LOW) {
      motor1_enc_count += 1;
    } else {
      motor1_enc_count -= 1;
    }
  } else {
    if (encA == LOW) {
      motor1_enc_count -= 1;
    } else {
      motor1_enc_count += 1;
    }
  }
}

/*--- MOTOR KF AND CONTROLLER ---*/

MotorKalmanFilter motor1_filter = MotorKalmanFilter();
MotorPID motor1_pid = MotorPID(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 300.0, 100.0, 0.0, 23.0, 1.5);

/*--- INPUT PARSER ---*/

// input: "[v] [w]\n" where [v] is linear vel and [w] is angular vel

String cmd;
void parseInput() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    // Serial.println(cmd);
    motor1_pid.driveMotor(cmd.toFloat());
  }
}

/*--- MAIN LOOP ---*/

void setup() {
  // setup encoder pins
  pinMode(MOTOR1_ENCA, INPUT);
  pinMode(MOTOR1_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCA), handleMotor1EncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCB), handleMotor1EncBChange, CHANGE);

  // initialize serial and wait to open
  Serial.begin(115200);
  while (!Serial) { ; }

  // initialize motors
  motor1_pid.begin();
}

void loop() {
  // initialize encoder count
  double prev_motor1_enc = motor1_enc_count;
  double motor1_vel;

  // initialize timer
  long prev_time = millis();
  double dt;

  // update state at 100 hz
  for (;;) {
    dt = (double)(millis() - prev_time) / 1000.0;
    prev_time = millis();

    // parse any incoming input
    parseInput();

    // filter update
    motor1_filter.update(dt, motor1_enc_count - prev_motor1_enc);

    // controller update
    motor1_vel = motor1_filter.getVel();

    // reset enc counts
    prev_motor1_enc = motor1_enc_count;

    // print velocity
    Serial.println(motor1_vel);

    delay(10);
  }
}
