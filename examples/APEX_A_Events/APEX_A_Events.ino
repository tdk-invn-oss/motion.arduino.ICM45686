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

const char* axis_str[3] = {"X", "Y", "Z"}; 
const char* direction_str[2] = {"+", "-"}; 
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

  // APEX Tilt enabled
  IMU.startTiltDetection();
  // APEX Pedometer enabled
  IMU.startPedometer();
  // APEX Raise to wake enabled
  IMU.startRaiseToWake();
  // APEX Tap enabled
  IMU.startTap();
  // APEX Freefall enabled  
  IMU.startFreeFall();
  // APEX HighG enabled
  IMU.startHighG();
  // APEX LowG enabled    
  IMU.startLowG();

  IMU.setApexInterrupt(2,irq_handler);
}

void loop() {
  // Wait for interrupt to read data Pedometer status
  if(irq_received) {
    irq_received = 0;
    uint8_t tap_count=0;
    uint8_t axis=0;
    uint8_t direction=0;
    char* activity;
    uint32_t step_count=0;
    float step_cadence=0;
    int raise_to_wake = 0;

    if(IMU.getTilt() == 1)
    {
      Serial.print("Tilt Event");
    }
    
    if(IMU.getPedometer(step_count,step_cadence,activity) == 1)
    {
      Serial.print(" Step count:");
      Serial.print(step_count);
      Serial.print(",");
      Serial.print("Step cadence(steps/s):");
      Serial.print(step_cadence);
      Serial.print(",");
      Serial.print("activity:");
      Serial.print(activity);
    }

    raise_to_wake = IMU.getRaiseToWake();

    if(raise_to_wake == 1)
    {
      Serial.print(" R2W Wake-up");
    } 
    else if (raise_to_wake == 2)
    {
      Serial.print(" R2W Sleep");
    }
    
    uint32_t duration_ms;  
    if(IMU.getFreefall(duration_ms) == 1)
    {
      Serial.print(" FreeFall Event duration(ms):");
      Serial.print(duration_ms);
    }

    if(IMU.getHighG() == 1)
      Serial.print(" HighG Event");

    if(IMU.getLowG() == 1)
      Serial.print(" LowG Event");

    if(IMU.getTap(tap_count,axis,direction) == 1)
    {
      Serial.print(" Tap count:");
      Serial.print(tap_count);
      Serial.print(",");
      Serial.print("Axis:");
      Serial.print(axis_str[axis]);
      Serial.print(",");
      Serial.print("Direction:");
      Serial.print(direction_str[direction]);
    }

    Serial.println("");
  }
}

