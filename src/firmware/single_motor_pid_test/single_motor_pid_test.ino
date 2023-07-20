#include <BasicLinearAlgebra.h>

/*--- MOTOR ENCODER ---*/

#define MOTOR_IN1 8
#define MOTOR_IN2 9
#define MOTOR_PWM 5
#define MOTOR_ENCA 3
#define MOTOR_ENCB 2
#define TICKS_PER_REV 680.0
#define WHEEL_DIAMETER 0.068

// from experimentation, seems to have ~680 counts per revolution
volatile long enc_count = 0;

// update count on ENCA change
void handleEncAChange() {
  int encA = digitalRead(MOTOR_ENCA);
  int encB = digitalRead(MOTOR_ENCB);

  if (encA == HIGH) {
    if (encB == HIGH) {
      enc_count += 1;
    } else {
      enc_count -= 1;
    }
  } else {
    if (encB == HIGH) {
      enc_count -= 1;
    } else {
      enc_count += 1;
    }
  }
}

// update count on ENCB change
void handleEncBChange() {
  int encA = digitalRead(MOTOR_ENCA);
  int encB = digitalRead(MOTOR_ENCB);

  if (encB == HIGH) {
    if (encA == LOW) {
      enc_count += 1;
    } else {
      enc_count -= 1;
    }
  } else {
    if (encA == LOW) {
      enc_count -= 1;
    } else {
      enc_count += 1;
    }
  }
}

/*--- KALMAN FILTER ---*/

// state variables
BLA::Matrix<2> state = { 0.0, 0.0 };  // [ang_pos, ang_vel]
BLA::Matrix<2, 2> state_cov = { 1.0, 0.0, 0.0, 1.0 };

// fixed error covariances
BLA::Matrix<2, 2> transition_error = { 0.01, 0.0, 0.0, 0.01 };
BLA::Matrix<1, 1> measurement_error = { 0.1 };

// transition and measurement models
BLA::Matrix<2, 2> A = { 1.0, 0.0, 0.0, 1.0 };  // A(0, 1) should be updated on update step
BLA::Matrix<1, 2> C = { 0.0, 0.0 };            // C(0, 1) should be updated on update step
BLA::Matrix<2, 2> I = { 1.0, 0.0, 0.0, 1.0 };

void filterUpdate(double dt, double enc_diff) {
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

/*--- PID + FEEDFORWARD CONTROL LOOP ---*/

#define PWM_MIN 10
#define PWM_MAX 255
#define INTEGRAL_DECAY 0.01

double PID_P = 100.0;
double PID_I = 100.0;  //0.25
double PID_D = 0.0;  //0.30
double PID_FF = 1.0; 

double integral = 0.0;
double derivative = 0.0;
double vel_target = 0.0;
double feed_forward = 0.0;
double diff = 0.0;

int motor_cmd = 0;

void pidUpdate(double dt) {
  if (abs(vel_target) > 0) {
    diff = (vel_target) - (state(1) * (WHEEL_DIAMETER / 2.0));
    derivative = diff / dt;
    integral += diff * dt;  // TODO: incorporate decay into this term
    feed_forward = 177 * vel_target - 74; // calculated experimentally
    motor_cmd = ((PID_FF * feed_forward) + (PID_P * diff) + (PID_I * integral) + (PID_D * derivative));
    motor_cmd = (motor_cmd / abs(motor_cmd)) * min(PWM_MAX, abs(motor_cmd));  // upper bound
  } else {
    motor_cmd = 0;
  }

  driveMotor(motor_cmd);
}

// TODO: add filtering to disallow 
void driveMotor(int motor_cmd) {
  // stop motor if motor cmd too low
  if (abs(motor_cmd) <= PWM_MIN) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (motor_cmd > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, motor_cmd);
  } else if (motor_cmd < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, motor_cmd);
  }
}

/*--- INPUT PARSER ---*/

String cmd;
void parseInput() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.println(cmd);
    if (cmd.charAt(0) == 'p') {
      PID_P = cmd.substring(2).toFloat();
    } else if (cmd.charAt(0) == 'i') {
      PID_I = cmd.substring(2).toFloat();
    } else if (cmd.charAt(0) == 'd') {
      PID_D = cmd.substring(2).toFloat();
    } else if (cmd.charAt(0) == 'v') {
      //integral = 0;
      vel_target = cmd.substring(2).toFloat();
    } else if (cmd.charAt(0) == 'm') {
      motor_cmd = cmd.substring(2).toInt();
      driveMotor(motor_cmd);
    }
  }
}

/*--- MAIN LOOP ---*/

void setup() {
  // setup motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  // setup encoder pins
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), handleEncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), handleEncBChange, CHANGE);

  // initialize serial and wait to open
  Serial.begin(115200);
  while (!Serial) { ; }

  // ensure motors are not moving
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

void loop() {
  // initialize encoder count
  double prev_enc = enc_count;

  // initialize timer
  long prev_time = millis();
  double dt;

  // update state at 100 hz
  for (;;) {
    dt = (double)(millis() - prev_time) / 1000.0;
    prev_time = millis();

    parseInput();
    pidUpdate(dt);
    filterUpdate(dt, enc_count - prev_enc);

    prev_enc = enc_count;

    Serial.print("P:");
    Serial.print(PID_P);
    Serial.print(" ");
    Serial.print("I:");
    Serial.print(PID_I);
    Serial.print(" ");
    Serial.print("D:");
    Serial.print(PID_D);
    Serial.print(" ");
    Serial.print("diff:");
    Serial.print(diff);
    Serial.print(" ");
    Serial.print("cmd:");
    Serial.print(motor_cmd);
    Serial.print(" ");
    Serial.print("v_t:");
    Serial.print(vel_target);
    Serial.print(" ");
    Serial.print("v:");
    Serial.println(state(1) * (WHEEL_DIAMETER / 2.0));

    delay(10);
  }
}
