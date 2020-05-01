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

#ifndef Globals_h
#define Globals_h

#include "def.h"
#include "Types.h"
#include "IMU.h"
#include <FS.h>

// Intermediate data handling variables
float                                   sensor_point[3];
int16_t                                 gyroADC[2][3];
int16_t                                 accADC[2][3];
int16_t                                 magADC[2][3];
int32_t                                 baroADC[2];

// Timing variables
uint32_t                                Begin;
uint32_t                                Acq_time;
uint32_t                                Start_time   = 0;
float                                   TimeStamp    = 0.0f;
uint32_t                                currentTime  = 0;
uint32_t                                previousTime = 0;
uint8_t                                 serial_input = 0;
uint32_t                                last_refresh = 0;
uint32_t                                delt_t       = 0;
uint32_t                                cycleTime    = 0;                                                                                                 // Main loop time (us)

// Calibration-related variables
full_adv_cal_t                          gyrocal[2];
full_adv_cal_t                          ellipsoid_magcal[2];
full_adv_cal_t                          accelcal[2];
full_adv_cal_t                          final_magcal[2];
uint8_t                                 GyroCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 EllipMagCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 AccelCal_buff[sizeof(full_adv_cal_t)];
uint8_t                                 FineMagCal_buff[sizeof(full_adv_cal_t)];
float                                   mag_calData[2][3];
float                                   dps_per_count = DPS_PER_COUNT;
float                                   g_per_count   = G_PER_COUNT;
float                                   UT_per_Count  = LIS2MDL_UT_PER_COUNT;
float                                   Mv_Cal        = 0.0f;
float                                   Mh_Cal        = 0.0f;
float                                   M_Cal         = 0.0f;
float                                   Del_Cal       = 0.0f;
uint8_t                                 cube_face     = 0;
uint8_t                                 face_rotation = 0;

// USFSMAX-related variables
CoProcessorConfig_t                     Cfg[2];
uint8_t                                 algostatus[2];
uint8_t                                 eventStatus[2];
int16_t                                 QT_Timestamp[2];                                                                                                  // USFSMAX Quaternion timestamps
uint8_t                                 cfg_buff[sizeof(CoProcessorConfig_t)];
uint8_t                                 EulerQuatFlag        = OUTPUT_EULER_ANGLES;
uint8_t                                 ScaledSensorDataFlag = SCALED_SENSOR_DATA;
volatile uint8_t                        data_ready[2]        = {1, 1};
uint8_t                                 cal_status[2]        = {0, 0};
uint8_t                                 gyroCalActive[2]     = {0, 0};
uint8_t                                 Quat_flag[2]         = {0, 0};                                                                                    // USFSMAX data ready flags
uint8_t                                 Gyro_flag[2]         = {0, 0};
uint8_t                                 Acc_flag[2]          = {0, 0};
uint8_t                                 Mag_flag[2]          = {0, 0};
uint8_t                                 Baro_flag[2]         = {0, 0};
float                                   Rsq                  = 0.0f;

// IMU-related variables
int16_t                                 accLIN[2][3];                                                                                                     // USFSMAX linear acceleration
int16_t                                 grav[2][3];
float                                   acc_LIN[2][3];                                                                                                    // 9DOF linear acceleration
float                                   Mx[2], My[2];                                                                                                     // Tilt-corrected horizontal magnetic components
float                                   gyroData[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   accData[2][3]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   magData[2][3]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float                                   qt[2][4]       = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};                                                // USFSMAX quaternions
float                                   QT[2][4]       = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};                                                // 9DOF quaternions
float                                   angle[2][2]    = {0.0f, 0.0f, 0.0f, 0.0f};                                                                        // USFSMAX P,R Euler angles
float                                   ANGLE[2][2]    = {0.0f, 0.0f, 0.0f, 0.0f};                                                                        // 9DOF P,R Euler angles
float                                   heading[2]     = {0.0f, 0.0f};                                                                                    // USFSMAX Heading Euler angle
float                                   HEADING[2]     = {0.0f, 0.0f};                                                                                    // 9DOF Heading Euler angle
float                                   GyroMeasError  = GYRO_MEAS_ERR*0.01745329252f;                                                                    // Madgewick fliter (RPS)
float                                   GyroMeasDrift  = GYRO_MEAS_ERR*0.01745329252f;                                                                    // Madgewick filter (RPS/s)
float                                   beta           = sqrt(3.0f/4.0f)*GyroMeasError;                                                                   // Madgewick fliter compute beta
float                                   zeta           = sqrt(3.0f/4.0f)*GyroMeasDrift;                                                                   // Madgewick fliter Compute zeta (Small or zero)
float                                   twoKp          = 2.0f*KP_DEF;                                                                                     // Mahony filter 2X proportional gain (Kp)
float                                   twoKi          = 2.0f*KI_DEF;                                                                                     // Mahony filter 2X integral gain (Ki)
float                                   integralFBx    = 0.0f;                                                                                            // Mahony filter integral error terms
float                                   integralFBy    = 0.0f;
float                                   integralFBz    = 0.0f;

#endif // Globals_h
