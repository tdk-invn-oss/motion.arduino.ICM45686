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
ICM456xx IMU(Wire,0);

volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

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
  // Enable interrupt on pin 2, Fifo watermark=10
  IMU.enableFifoInterrupt(2,irq_handler,10);
  // Accel ODR = 100 Hz and Full Scale Range = 16G
  IMU.startAccel(100,16);
  // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
  IMU.startGyro(100,2000);

}

void loop() {
  // Wait for interrupt to read data from fifo
  if(irq_received) {
    irq_received = 0;
    for(int i = 0; i < 10 ; i ++)
    {
      inv_imu_fifo_data_t imu_data;
      IMU.getDataFromFifo(imu_data);
      // Format data for Serial Plotter
      Serial.print("AccelX:");
      Serial.println(imu_data.byte_16.accel_data[0]);
      Serial.print("AccelY:");
      Serial.println(imu_data.byte_16.accel_data[1]);
      Serial.print("AccelZ:");
      Serial.println(imu_data.byte_16.accel_data[2]);
      Serial.print("GyroX:");
      Serial.println(imu_data.byte_16.gyro_data[0]);
      Serial.print("GyroY:");
      Serial.println(imu_data.byte_16.gyro_data[1]);
      Serial.print("GyroZ:");
      Serial.println(imu_data.byte_16.gyro_data[2]);
      Serial.print("Temperature:");
      Serial.println(imu_data.byte_16.temp_data);
    }
  }
}
