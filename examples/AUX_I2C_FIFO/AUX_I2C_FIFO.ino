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

// Instantiate an ICM456XX with LSB address set to 0
ICM456xx IMU(SPI,0);

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
  
  delay(2000);
  IMU.startAccel(25,16);
  IMU.startGyro(25,2000);
  ret = IMU.setI2CM_FIFO(2,irq_handler);
}

void loop() {
  // Wait for interrupt to read data from fifo
  if(irq_received) {
    int32_t accel[3], gyro[3];
    float external[3] ={0,0,0};
    int ret = 0;
    irq_received = 0;
    ret = IMU.getAdvDataFromFifo(accel, gyro, external);

    Serial.print("AccelX:");
    Serial.print(accel[0]);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(accel[1]);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(accel[2]);
    
    Serial.print(" GyroX:");
    Serial.print(gyro[0]);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(gyro[1]);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(gyro[2]);

    Serial.print(" MagX:");
    Serial.print(external[0]);
    Serial.print(",");
    Serial.print("MagY:");
    Serial.print(external[1]);
    Serial.print(",");
    Serial.print("MagZ:");
    Serial.println(external[2]);
  }
}
