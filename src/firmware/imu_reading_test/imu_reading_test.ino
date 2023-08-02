#include <Adafruit_MPU6050.h>

#define MPU_ROT_X_OFFSET 0.0494
#define MPU_ROT_Y_OFFSET -0.0049
#define MPU_ROT_Z_OFFSET 0.0132

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);

  // wait for MPU-6050 to initialize
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

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("GyroX:");
  Serial.print(g.gyro.x + MPU_ROT_X_OFFSET);
  Serial.print(", ");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y + MPU_ROT_Y_OFFSET);
  Serial.print(", ");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z + MPU_ROT_Z_OFFSET);
  Serial.println("");

  delay(10);
}