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
 
#include "ICM45686.h"

// Instantiate an ICM456XX with SPI interface and CS on pin 8
ICM456xx IMU(SPI,8);

#define ACCEL_FSR_G    16			/* The accel FSR is 16g */
#define GYRO_FSR_DPS   2000			/* The gyro FSR is 2000 dps */
#define MAX_LSB        32768
#define RAW_MAG_SCALE  0.075

volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

void setup() {
  int ret;
  uint8_t data;
  
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICM456XX
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }
  
  IMU.startAccel(25,ACCEL_FSR_G);
  IMU.startGyro(25,GYRO_FSR_DPS);
  ret = IMU.setI2CM_FIFO(2,irq_handler);
}

void loop() {
  // Wait for interrupt to read data from fifo
  if(irq_received) {
    int32_t accel[3], gyro[3], external[3];
    float data[3] = { 0 };
    int ret = 0;
    irq_received = 0;
    ret = IMU.getAdvDataFromFifo(accel, gyro, external);

    data[0]  = (float)(accel[0] * ACCEL_FSR_G) / MAX_LSB;
    data[1]  = (float)(accel[1] * ACCEL_FSR_G) / MAX_LSB;
    data[2]  = (float)(accel[2] * ACCEL_FSR_G) / MAX_LSB;    

    // Print accel raw to g
    Serial.print("AccelX:"); Serial.print(data[0]); Serial.print(",");
    Serial.print("AccelY:"); Serial.print(data[1]); Serial.print(",");
    Serial.print("AccelZ:"); Serial.print(data[2]);
    
    data[0]  = (float)(gyro[0] * GYRO_FSR_DPS) / MAX_LSB;
    data[1]  = (float)(gyro[1] * GYRO_FSR_DPS) / MAX_LSB;
    data[2]  = (float)(gyro[2] * GYRO_FSR_DPS) / MAX_LSB;   
    
    // Print gyro raw to dsp
    Serial.print(" GyroX:"); Serial.print(data[0]); Serial.print(",");
    Serial.print("GyroY:"); Serial.print(data[1]); Serial.print(",");
    Serial.print("GyroZ:"); Serial.print(data[2]);

    data[0]  = ((float)external[0] * RAW_MAG_SCALE)/16.0f;
    data[1]  = ((float)external[1] * RAW_MAG_SCALE)/16.0f;
    data[2]  = ((float)external[2] * RAW_MAG_SCALE)/16.0f;  

    // print mag raw to uT
    Serial.print(" MagX:"); Serial.print(data[0]); Serial.print(",");
    Serial.print("MagY:"); Serial.print(data[1]); Serial.print(",");
    Serial.print("MagZ:"); Serial.println(data[2]);
  }
}
