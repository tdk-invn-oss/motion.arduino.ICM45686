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
  // Accel ODR = 50 Hz and APEX Pedometer enabled
  IMU.startPedometer(2,irq_handler);

}

void loop() {
  // Wait for interrupt to read data Pedometer status
  if(irq_received) {
    uint32_t step_count=0;
    float step_cadence=0;
    char* activity;
    irq_received = 0;
    IMU.getPedometer(step_count,step_cadence,activity);
    Serial.print("Step_count:");
    Serial.print(step_count);
    Serial.print(",");
    Serial.print("Step_cadence:");
    Serial.print(step_cadence);
    Serial.print(",");
    Serial.print("activity:");
    Serial.print(activity);
    Serial.println("");
  }
}
