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

#ifndef Sensor_cal_h
#define Sensor_cal_h

#include "Alarms.h"
#include "Types.h"
#include "def.h"
#include "config.h"
#include "USFSMAX.h"

extern uint8_t                          Acc_flag[2], Mag_flag[2];
extern full_adv_cal_t                   gyrocal[2];
extern full_adv_cal_t                   accelcal[2];
extern full_adv_cal_t                   ellipsoid_magcal[2];
extern full_adv_cal_t                   final_magcal[2];
extern float                            sensor_point[3];
extern float                            gyroData[2][3];
extern float                            accData[2][3];
extern float                            magData[2][3];
extern uint8_t                          gyroCalActive[2];
extern volatile uint8_t                 data_ready[2];

class Sensor_cal
{
  public:
                                        Sensor_cal(I2Cdev*, USFSMAX*, uint8_t);
     void                               GyroCal();
     void                               apply_adv_calibration(full_adv_cal_t calibration, int16_t *raw, float sf, float *out);
     void                               apply_adv_calibration(full_adv_cal_t calibration, float *raw, float sf, float *out);
     void                               sendOneToProceed();
  private:
     I2Cdev*                            _i2c;
     USFSMAX*                           _usfsmax;
     uint8_t                            _sensornum;
};

#endif // Sensor_cal_h
