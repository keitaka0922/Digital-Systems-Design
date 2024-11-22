// MPU-6050 Short Example Sketch
#include<Wire.h>
#include <MPU6050.h>


const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  
  // Initialize MPU6050
  mpu.initialize();

  // Check connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }
  Serial.println("MPU6050 connected!");

  // Perform CALIBRATION
  calibrateMPU();
  Serial.println("Calibration complete!");
}
void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print(AcX); Serial.print(",");
  Serial.print(AcY); Serial.print(","); 
  Serial.print(AcZ); Serial.print(",");
  Serial.print(Tmp / 340.00 + 36.53); Serial.print(","); // Convert temperature
  Serial.print(GyX); Serial.print(",");
  Serial.print(GyY); Serial.print(",");
  Serial.println(GyZ);
  
  // Normalize the raw data
  float norm = sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ);
  float Ax_norm = AcX / norm;
  float Ay_norm = AcY / norm;
  float Az_norm = AcZ / norm;

  // Calculate pitch and roll in radians
  float pitch = atan2(Ax_norm,Az_norm);               // Pitch (up/down tilt)
  float roll = atan2(Ay_norm, Az_norm);      // Roll (side-to-side tilt)

  // Convert pitch and roll to degrees
  pitch = pitch * 180 / PI;
  roll = roll * 180 / PI;

  // Output the results
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("°\tRoll: ");
  Serial.print(roll);
  Serial.println("°");

  delay(1000); // Wait for 1 second before the next reading

  // Determine sleep position based on thresholds
  if (abs(roll) < 15 && pitch > -20 && pitch < 20) {
      Serial.println("Position: Back Sleeping");
  } else if (abs(roll) > 30 && pitch > -20 && pitch < 20) {
      Serial.println("Position: Side Sleeping");
  } else if (abs(roll) < 15 && pitch > 20) {
      Serial.println("Position: Stomach Sleeping");
  } else {
      Serial.println("Position: Unknown/Transition");
  }

  
}

// CALIBRATION

void calibrateMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  const int samples = 1000; // Number of samples for averaging

  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axSum += ax;
    aySum += ay;
    azSum += az - 16384; // Subtract 16384 for gravity (1 g)
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(2); // Small delay between readings
  }

  // Calculate offsets
  axOffset = axSum / samples;
  ayOffset = aySum / samples;
  azOffset = azSum / samples;
  gxOffset = gxSum / samples;
  gyOffset = gySum / samples;
  gzOffset = gzSum / samples;

  Serial.println("Offsets calculated:");
  Serial.print("Accel: ");
  Serial.print(axOffset); Serial.print(", ");
  Serial.print(ayOffset); Serial.print(", ");
  Serial.println(azOffset);
  Serial.print("Gyro: ");
  Serial.print(gxOffset); Serial.print(", ");
  Serial.print(gyOffset); Serial.print(", ");
  Serial.println(gzOffset);
}
