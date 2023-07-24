#define MOTOR_ENCA 2
#define MOTOR_ENCB 1

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

void setup() {
  // setup encoder pins
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), handleEncAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), handleEncBChange, CHANGE);

  // initialize serial and wait to open
  Serial.begin(9600);
  while (!Serial) {;}
}

void loop() {
  // continuously print encoder value
  Serial.println(enc_count);
  delay(100);
}
