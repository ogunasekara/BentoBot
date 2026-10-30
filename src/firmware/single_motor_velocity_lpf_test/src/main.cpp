#include "Arduino.h"

/*--- ENCODER ---*/

#define MOTOR_ENCA 11
#define MOTOR_ENCB 12
#define MOTOR_IN1 6
#define MOTOR_IN2 5
#define MOTOR_PWM 7

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

/*--- LOW PASS FILTER ---*/

double vel_estimate = 0;
double alpha = 0.2;
void filterUpdate(double dt, double enc_diff) {
    double vel_raw = (enc_diff / dt) * (2 * PI / TICKS_PER_REV);
    vel_estimate += alpha * (vel_raw - vel_estimate);
}

/*--- INPUT PARSING ---*/

String cmd;
void parseInput() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();

    float vel_cmd = cmd.toFloat();
    
    if (vel_cmd == 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, 0);
    } else if (vel_cmd > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, abs(vel_cmd));
    } else if (vel_cmd < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_PWM, abs(vel_cmd));
    }
  }
}

/*--- MAIN LOOP ---*/

void setup() {
  // setup encoder pins
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), handleEncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), handleEncBChange, CHANGE);

  // setup motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // ensure motors are not moving
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);

  // initialize serial and wait to open
  Serial.begin(9600);
  while (!Serial) { ; }
}

void loop() {
  // initialize encoder count
  double prev_enc = enc_count;

  // update state at 100 hz
  for(;;) {
    parseInput();
    filterUpdate(0.01, enc_count - prev_enc);
    prev_enc = enc_count;

    Serial.println(vel_estimate);

    delay(10);
  }
}
