#include <BasicLinearAlgebra.h>

/*--- ENCODER ---*/

#define MOTOR_ENCA 2
#define MOTOR_ENCB 1
#define TICKS_PER_REV 680.0

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
BLA::Matrix<2, 2> A = { 1.0, 0.0, 0.0, 1.0 }; // A(0, 1) should be updated on update step
BLA::Matrix<1, 2> C = { 0.0, 0.0 }; // C(0, 1) should be updated on update step
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

/*--- MAIN LOOP ---*/

void setup() {
  // setup encoder pins
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), handleEncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), handleEncBChange, CHANGE);

  // initialize serial and wait to open
  Serial.begin(9600);
  while (!Serial) { ; }
}

void loop() {
  // initialize encoder count
  double prev_enc = enc_count;

  // update state at 100 hz
  for(;;) {
    filterUpdate(0.01, enc_count - prev_enc);
    prev_enc = enc_count;

    Serial.print(state(0));
    Serial.print(", ");
    Serial.println(state(1));

    delay(10);
  }
}
