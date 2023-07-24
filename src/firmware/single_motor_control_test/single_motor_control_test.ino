// define wiring to arduino
#define MOTOR_IN1 3
#define MOTOR_IN2 4
#define MOTOR_PWM 5

// motor properties
#define MIN_PWM 0
#define MAX_PWM 255

void setup() {
  // setup motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  // initialize system as off
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_PWM, LOW);
}

// loop the PWM values from min to max and back again
void loopSpeeds() {
  analogWrite(MOTOR_PWM, 0);
  for (int val = MIN_PWM; val <= MAX_PWM; val++) {
    analogWrite(MOTOR_PWM, val);
    delay(20);
  }

  for (int val = MAX_PWM; val >= MIN_PWM; val--) {
    analogWrite(MOTOR_PWM, val);
    delay(20);
  }
  analogWrite(MOTOR_PWM, 0);
}

void loop() {
  // spin in one direction
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  loopSpeeds();

  delay(2000);

  // spin in other direction
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  loopSpeeds();

  delay(2000);
}
