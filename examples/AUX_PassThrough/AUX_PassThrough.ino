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

#define ICT1531X_CHIP_ID_REG  0x01

// Instantiate an ICM456XX with LSB address set to 0
ICM456xx IMU(Wire,0);

volatile uint8_t irq_received = 0;

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
  ret = IMU.setI2CMPassThrough();
  ret |= IMU.getDataFromPassThrough(ICT1531X_CHIP_ID_REG, data);
  Serial.print("Read data: ");
  Serial.println(data);
}

void loop() {
  // Wait for interrupt to read data from fifo
  if(irq_received) {
    irq_received = 0;
  }
}
