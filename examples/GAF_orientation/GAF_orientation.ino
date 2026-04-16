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

/* 
  The Arduino Serial Plotter can display a maximum of eight values or curves.
  So you can print one sensor data except algo output
*/
//#define PRINT_ACCEL
//#define PRINT_GYRO
//#define PRINT_MAG
//#define PRINT_BIAS

// Instantiate an ICM456XX with LSB address set to 0
ICM456xx IMU(Wire, 0);

uint8_t irq_received = 0;
uint8_t algo = ALGO_GRV;

void irq_handler(void) {
  irq_received = 1;
}

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  // Initializing the ICM456XX
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  }
  // Start GAF algo with interrupt on pin 2
  // ALGO_GRV, enable GRV when enable 6-axis(AG)
  // ALGO_GMRV, enable GMRV when enable 6-axis(AM)
  // ALGO_RV, enable RV when enable 9-axis(AGM)
  algo = ALGO_GRV;
  ret = IMU.startGaf(2, irq_handler, algo);
  if (ret != 0) {
    Serial.print("GAF failed: ");
    Serial.println(ret);
    while (1)
      ;
  }

  Serial.println("CLEARDATA");
}

void loop() {
  // Wait for interrupt to read data from fifo
  if (irq_received) {
    irq_received = 0;

    float W, X, Y, Z, accuracy;

    if (algo == ALGO_GRV)
    {
      IMU.getGaf_GRVData(W, X, Y, Z);
      Serial.print("GRV ");
    } else if (algo == ALGO_GMRV)
    {
      IMU.getGaf_GMRVData(W, X, Y, Z, accuracy);
      Serial.print("GMRV ");
    } else if (algo == ALGO_RV)
    {
      IMU.getGaf_RVData(W, X, Y, Z, accuracy);
      Serial.print("RV ");
    }

    Serial.print("W:");
    Serial.print(W);
    Serial.print(",");
    Serial.print("X:");
    Serial.print(X);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(Y);
    Serial.print(",");
    Serial.print("Z:");
    Serial.print(Z); 
    if (algo != ALGO_GRV){
      Serial.print(",");
      Serial.print("Accuracy:");
      Serial.print(accuracy);
    }
    Serial.print(" ");
    
#ifdef PRINT_BIAS
    int bx,by,bz,gaf_accuracy;
    // Read bias(q16) and accuracy for gyro(GYRO), mag(MAG)
    IMU.getGaf_BiasData(GYRO, bx, by, bz, gaf_accuracy);
    Serial.print("Bias_X:");
    Serial.print(bx);
    Serial.print(",");
    Serial.print("Bias_Y:");      
    Serial.print(by);
    Serial.print(",");
    Serial.print("Bias_Z:");      
    Serial.print(bz);
    Serial.print(",");
    Serial.print("Accuracy:");
    Serial.print(gaf_accuracy);
    Serial.print(" ");
#endif

#ifdef PRINT_ACCEL
    {
      float accel_cal_x, accel_cal_y, accel_cal_z;
      // Read accel data
      IMU.getCalibratedAccel(accel_cal_x, accel_cal_y, accel_cal_z);
      Serial.print("Cal_AccelX:");
      Serial.print(accel_cal_x);
      Serial.print(",");
      Serial.print("Cal_AccelY:");
      Serial.print(accel_cal_y);
      Serial.print(",");
      Serial.print("Cal_AccelZ:");
      Serial.print(accel_cal_z);
      Serial.print(" ");
    }
#endif

#ifdef PRINT_GYRO
    if (algo != ALGO_GMRV) {
      float gyro_cal_x, gyro_cal_y, gyro_cal_z;
      // Read calibrated gyro data
      IMU.getCalibratedGyro(gyro_cal_x, gyro_cal_y, gyro_cal_z);
      Serial.print("Cal_GyroX:");
      Serial.print(gyro_cal_x);
      Serial.print(",");
      Serial.print("Cal_GyroY:");      
      Serial.print(gyro_cal_y);
      Serial.print(",");
      Serial.print("Cal_GyroZ:");      
      Serial.print(gyro_cal_z);
      Serial.print(" ");
    }
#endif

#ifdef PRINT_MAG
    if (algo == ALGO_GMRV || algo == ALGO_RV) {
      float mag_cal_x, mag_cal_y, mag_cal_z;
      // Read calibrated mag data
      IMU.getCalibratedMag(mag_cal_x, mag_cal_y, mag_cal_z);
      Serial.print("Cal_MagX:");
      Serial.print(mag_cal_x);
      Serial.print(",");
      Serial.print("Cal_MagY:");
      Serial.print(mag_cal_y);
      Serial.print(",");
      Serial.print("Cal_MagZ:");
      Serial.print(mag_cal_z);
    }
#endif
    Serial.println("");
  }
}
