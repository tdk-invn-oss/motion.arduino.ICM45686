/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#include "ICM45686S.h"

#define ACCEL_FSR_G    16				/* The accel FSR is 16g */
#define GYRO_FSR_DPS   2000			/* The gyro FSR is 2000 dps */
#define MAX_LSB        32768

// Instantiate an ICM456XX with LSB address set to 0
ICM456xx IMU(Wire,0);

void setup() {
  int ret;
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICM456XX
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  IMU.startAccel(100,ACCEL_FSR_G);
  IMU.startGyro(100,GYRO_FSR_DPS);
  // Wait IMU to start
  delay(100);
}

void loop() {

  inv_imu_sensor_data_t imu_data;
  float accel_g[3] = { 0 };
  float gyro_dps[3] = { 0 };
  float temp_degc;

  // Read registers
  IMU.getDataFromRegisters(imu_data);

  // Format data for Serial Plotter
  accel_g[0]  = (float)(imu_data.accel_data[0] * ACCEL_FSR_G) / MAX_LSB;
  accel_g[1]  = (float)(imu_data.accel_data[1] * ACCEL_FSR_G) / MAX_LSB;
  accel_g[2]  = (float)(imu_data.accel_data[2] * ACCEL_FSR_G) / MAX_LSB;

  // Format data for Serial Plotter
  Serial.print("AccelX:");
  Serial.print(accel_g[0]);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accel_g[1]);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accel_g[2]);
  Serial.print(",");

  gyro_dps[0]  = (float)(imu_data.gyro_data[0] * GYRO_FSR_DPS) / MAX_LSB;
  gyro_dps[1]  = (float)(imu_data.gyro_data[1] * GYRO_FSR_DPS) / MAX_LSB;
  gyro_dps[2]  = (float)(imu_data.gyro_data[2] * GYRO_FSR_DPS) / MAX_LSB;

  Serial.print("GyroX:");
  Serial.print(gyro_dps[0]);
  Serial.print(",");  
  Serial.print("GyroY:");
  Serial.print(gyro_dps[1]);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(gyro_dps[2]);
  Serial.print(",");

  temp_degc = 25 + ((float)imu_data.temp_data/128);

  Serial.print("Temperature:");
  Serial.print(temp_degc);
  Serial.println("");

  // Run @ ODR 100Hz
  delay(10);
}
