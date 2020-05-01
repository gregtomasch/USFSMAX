/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "Sensor_cal.h"

Sensor_cal::Sensor_cal(I2Cdev* i2c, USFSMAX* usfsmax, uint8_t sensornum)
{
  _i2c       = i2c;
  _usfsmax   = usfsmax;
  _sensornum = sensornum;
}

void Sensor_cal::GyroCal()
{
  Alarms::blueLEDoff();
  _i2c->writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x01);                                                                               // 0x01 - assert bit 0, start gyro cal
  gyroCalActive[_sensornum] = 1;
}

void Sensor_cal::sendOneToProceed()
{
  uint8_t input = 0;
  
  Serial.println("Send '1' to continue...");
  while(1)
  {
    input = Serial.read();
    if(input == 49) break;
    delay(10);
  }
}

void Sensor_cal::apply_adv_calibration(full_adv_cal_t calibration, int16_t *raw, float sf, float *out)
{
  float x, y, z;

  x = ((float)raw[0] * sf) - calibration.V[0];
  y = ((float)raw[1] * sf) - calibration.V[1];
  z = ((float)raw[2] * sf) - calibration.V[2];
  out[0] = (x * calibration.invW[0][0] + y * calibration.invW[0][1] + z * calibration.invW[0][2]);
  out[1] = (x * calibration.invW[1][0] + y * calibration.invW[1][1] + z * calibration.invW[1][2]);
  out[2] = (x * calibration.invW[2][0] + y * calibration.invW[2][1] + z * calibration.invW[2][2]);
}

void Sensor_cal::apply_adv_calibration(full_adv_cal_t calibration, float *raw, float sf, float *out)
{
  float x, y, z;

  x = (raw[0] * sf) - calibration.V[0];
  y = (raw[1] * sf) - calibration.V[1];
  z = (raw[2] * sf) - calibration.V[2];
  out[0] = (x * calibration.invW[0][0] + y * calibration.invW[0][1] + z * calibration.invW[0][2]);
  out[1] = (x * calibration.invW[1][0] + y * calibration.invW[1][1] + z * calibration.invW[1][2]);
  out[2] = (x * calibration.invW[2][0] + y * calibration.invW[2][1] + z * calibration.invW[2][2]);
}
