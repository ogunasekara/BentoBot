#include <BasicLinearAlgebra.h>
#include <Adafruit_MPU6050.h>
#include <MotorKalmanFilter.h>
#include <MotorPID.h>

/*--- IMU ---*/

#define MPU_ROT_X_OFFSET 0.0582
#define MPU_ROT_Y_OFFSET -0.0037
#define MPU_ROT_Z_OFFSET 0.0461

Adafruit_MPU6050 mpu;

void initializeIMU() {
  // initialize IMU
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU-6050 chip");
    while (1) {
      delay(10);
    }
  }

  // set desired properties for sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

/*--- MOTOR ENCODERS ---*/

// left motor
#define MOTOR1_ENCA 11
#define MOTOR1_ENCB 12
#define MOTOR1_IN1 6
#define MOTOR1_IN2 5
#define MOTOR1_PWM 7

// right motor
#define MOTOR2_ENCA 10
#define MOTOR2_ENCB 9
#define MOTOR2_IN1 4
#define MOTOR2_IN2 3
#define MOTOR2_PWM 2

#define TICKS_PER_REV 680.0 // found through experimentation
#define WHEEL_DIAMETER 0.068
#define WHEEL_BASE_LENGTH 0.16

volatile long motor1_enc_count = 0;
volatile long motor2_enc_count = 0;

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

void handleMotor2EncAChange() {
  int encA = digitalRead(MOTOR2_ENCA);
  int encB = digitalRead(MOTOR2_ENCB);

  if (encA == HIGH) {
    if (encB == HIGH) {
      motor2_enc_count += 1;
    } else {
      motor2_enc_count -= 1;
    }
  } else {
    if (encB == HIGH) {
      motor2_enc_count -= 1;
    } else {
      motor2_enc_count += 1;
    }
  }
}

void handleMotor2EncBChange() {
  int encA = digitalRead(MOTOR2_ENCA);
  int encB = digitalRead(MOTOR2_ENCB);

  if (encB == HIGH) {
    if (encA == LOW) {
      motor2_enc_count += 1;
    } else {
      motor2_enc_count -= 1;
    }
  } else {
    if (encA == LOW) {
      motor2_enc_count -= 1;
    } else {
      motor2_enc_count += 1;
    }
  }
}

/*--- MOTOR KF AND CONTROLLER ---*/

MotorKalmanFilter motor1_filter = MotorKalmanFilter();
MotorPID motor1_pid = MotorPID(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 300.0, 100.0, 0.0, 64.4, 0.807);

MotorKalmanFilter motor2_filter = MotorKalmanFilter();
MotorPID motor2_pid = MotorPID(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, 300.0, 100.0, 0.0, 71.2, 0.774);

/*--- INPUT PARSER ---*/

// input: "[v] [w]\n" where [v] is linear vel and [w] is angular vel

String cmd;
void parseInput() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.println(cmd);

    int ind = cmd.indexOf(' ');
    if ( ind == -1 ) { return; }

    double lin_vel = cmd.substring(0, ind).toFloat();
    double ang_vel = cmd.substring(ind + 1).toFloat();

    double left_vel = (2.0 * lin_vel - WHEEL_BASE_LENGTH * ang_vel) / 2.0;
    double right_vel = (2.0 * lin_vel + WHEEL_BASE_LENGTH * ang_vel) / 2.0;

    motor1_pid.setVel(left_vel);
    motor2_pid.setVel(right_vel);
  }
}

/*--- MAIN LOOP ---*/

void setup() {
  // setup encoder pins
  pinMode(MOTOR1_ENCA, INPUT);
  pinMode(MOTOR1_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCA), handleMotor1EncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCB), handleMotor1EncBChange, CHANGE);

  pinMode(MOTOR2_ENCA, INPUT);
  pinMode(MOTOR2_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCA), handleMotor2EncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCB), handleMotor2EncBChange, CHANGE);

  // initialize serial and wait to open
  Serial.begin(115200);
  while (!Serial) { ; }

  // initialize motors
  motor1_pid.begin();
  motor2_pid.begin();

  // initialize IMU
  initializeIMU();
}

void loop() {
  // initialize encoder count
  double prev_motor1_enc = motor1_enc_count;
  double prev_motor2_enc = motor2_enc_count;
  double motor1_vel, motor2_vel;
  double lin_vel, ang_vel;

  // initialize timer
  long prev_time = millis();
  double dt;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;

  // update state at 100 hz
  for (;;) {
    dt = (double)(millis() - prev_time) / 1000.0;
    prev_time = millis();

    // parse any incoming input
    parseInput();

    // filter update
    motor1_filter.update(dt, motor1_enc_count - prev_motor1_enc);
    motor2_filter.update(dt, motor2_enc_count - prev_motor2_enc);

    // controller update
    motor1_vel = motor1_filter.getVel();
    motor2_vel = motor2_filter.getVel();
    motor1_pid.update(dt, motor1_vel);
    motor2_pid.update(dt, motor2_vel);

    // reset enc counts
    prev_motor1_enc = motor1_enc_count;
    prev_motor2_enc = motor2_enc_count;

    lin_vel = (motor2_vel + motor1_vel) / 2.0;
    ang_vel = (motor2_vel - motor1_vel) / WHEEL_BASE_LENGTH;

    // get IMU information
    mpu.getEvent(&a, &g, &temp);

    Serial.print(motor1_vel);
    Serial.print(" ");
    Serial.print(motor2_vel);
    Serial.print(" ");
    Serial.println(g.gyro.z + MPU_ROT_Z_OFFSET);

    delay(10);
  }
}
