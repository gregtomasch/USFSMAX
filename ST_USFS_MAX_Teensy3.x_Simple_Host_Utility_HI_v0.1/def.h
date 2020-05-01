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

#ifndef def_h
#define def_h

#include "config.h"

/*************************************************************************************************/
/*************                                                                     ***************/
/*************              IMU Orientations and Sensor Definitions                ***************/
/*************                                                                     ***************/
/*************************************************************************************************/
#if defined(USFS_MAX)
  
  // AHRS filters follow ENU convention          EAST                                 NORTH                               UP
  #define ACC_ORIENTATION(X, Y, Z)              {accData[sensorNUM][EAST]   = +X;     accData[sensorNUM][NORTH] = +Y;     accData[sensorNUM][UP]   = +Z;}

  // ENU rotation axes                           PITCH (EAST) axis gyro (nose-up +)   ROLL (NORTH) axis gyro (CW +)       YAW (UP) axis gyro (E of N +)
  #define GYRO_ORIENTATION(X, Y, Z)             {gyroData[sensorNUM][PITCH] = +X;     gyroData[sensorNUM][ROLL] = +Y;     gyroData[sensorNUM][YAW] = -Z;}
  
  // Mag follows NED convention                  FWD                                  RIGHT                               DOWN                             // Check component sign with NOAA calculator
  #define MAG_ORIENTATION(X, Y, Z)              {magData[sensorNUM][0]      = -Y;     magData[sensorNUM][1]     = +X;     magData[sensorNUM][2]    = -Z;}  // Z is multiplied by an additional factor of -1 because this Goddam sensor is left-handed!
#endif

#if defined(LSM6DSM_GYRO_LPF_167) || defined(LSM6DSM_GYRO_LPF_223) || defined(LSM6DSM_GYRO_LPF_314) \
    || defined(LSM6DSM_GYRO_LPF_655)
  
  #if defined(LSM6DSM_GYRO_LPF_167)
    #define LSM6DSM_GYRO_DLPF_CFG               0x02
  #endif
  #if defined(LSM6DSM_GYRO_LPF_223)
    #define LSM6DSM_GYRO_DLPF_CFG               0x01
  #endif
  #if defined(LSM6DSM_GYRO_LPF_314)
    #define LSM6DSM_GYRO_DLPF_CFG               0x00
  #endif
  #if defined(LSM6DSM_GYRO_LPF_655)
    #define LSM6DSM_GYRO_DLPF_CFG               0x03
  #endif
  #else
    //Default settings LPF 314Hz
    #define LSM6DSM_GYRO_DLPF_CFG               0x00
#endif
  
#if defined(LSM6DSM_ACC_LPF_ODR_DIV2) || defined(LSM6DSM_ACC_LPF_ODR_DIV4) || defined(LSM6DSM_ACC_LPF_ODR_DIV9) \
    || defined(LSM6DSM_ACC_LPF_ODR_DIV50) || defined(LSM6DSM_ACC_LPF_ODR_DIV100) || defined(LSM6DSM_ACC_LPF_ODR_DIV400)
  
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV2)
    #define LSM6DSM_ACC_DLPF_CFG                0x00
  #endif
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV4)
    #define LSM6DSM_ACC_DLPF_CFG                0x01
  #endif
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV9)
    #define LSM6DSM_ACC_DLPF_CFG                0x02
  #endif
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV50)
    #define LSM6DSM_ACC_DLPF_CFG                0x03
  #endif
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV100)
    #define LSM6DSM_ACC_DLPF_CFG                0x04
  #endif
  #if defined(LSM6DSM_ACC_LPF_ODR_DIV400)
    #define LSM6DSM_ACC_DLPF_CFG                0x05
  #endif
#else
    //Default settings LPF ODR/9
    #define LSM6DSM_ACC_DLPF_CFG                0x02
#endif
  
#if defined(ACC_SCALE_2) || defined(ACC_SCALE_4) || defined(ACC_SCALE_8) || defined(ACC_SCALE_16)

  #if defined(ACC_SCALE_2)
    #define ACC_SCALE                           0x00
    #define G_PER_COUNT                         0.0000610f
  #endif
  #if defined(ACC_SCALE_4)
    #define ACC_SCALE                           0x02
    #define G_PER_COUNT                         0.0001220f
  #endif
  #if defined(ACC_SCALE_8)
    #define ACC_SCALE                           0x03
    #define G_PER_COUNT                         0.0002440f
  #endif
  #if defined(ACC_SCALE_16)
    #define ACC_SCALE                           0x01
    #define G_PER_COUNT                         0.0004880f
  #endif
#endif

#if defined(GYRO_SCALE_125) || defined(GYRO_SCALE_250) || defined(GYRO_SCALE_500) || defined(GYRO_SCALE_1000) \
    || defined(GYRO_SCALE_2000)

  #if defined(GYRO_SCALE_125)
    #define GYRO_SCALE                          0x02
    #define DPS_PER_COUNT                       0.004375f
  #endif
  #if defined(GYRO_SCALE_250)
    #define GYRO_SCALE                          0x00
    #define DPS_PER_COUNT                       0.00875f
  #endif
  #if defined(GYRO_SCALE_500)
    #define GYRO_SCALE                          0x04
    #define DPS_PER_COUNT                       0.0175f
  #endif
  #if defined(GYRO_SCALE_1000)
    #define GYRO_SCALE                          0x08
    #define DPS_PER_COUNT                       0.035f
 #endif
  #if defined(GYRO_SCALE_2000)
    #define GYRO_SCALE                          0x0C
    #define DPS_PER_COUNT                       0.070f
  #endif
#endif

#define LIS2MDL_UT_PER_COUNT                    0.15f
#define RPS_PER_DPS                             0.01745329

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                         Magnetic Constants                          ***************/
/*************                                                                     ***************/
/*************************************************************************************************/
/* Geomagnetic field data: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
   Units are uT, angles are in decimal degrees (Not DMS)*/

#ifdef KELSEYVILLE_CA_USA
  #define M_V                                   42.9631f
  #define M_H                                   22.7568f
  #define MAG_DECLINIATION                      13.7433f
#endif
#ifdef DANVILLE_CA_USA
  #define M_V                                   42.2089f
  #define M_H                                   23.0939f
  #define MAG_DECLINIATION                      13.2197f
#endif
#ifdef YUBA_CITY_CA_USA
  #define M_V                                   43.3972f
  #define M_H                                   22.6066f
  #define MAG_DECLINIATION                      13.5336f
#endif
#ifdef SUNNYVALE_CA_USA
  #define M_V                                   41.8128f
  #define M_H                                   23.2519f
  #define MAG_DECLINIATION                      13.2197f
#endif
#ifdef MISSISSAUGA_ON_CA
  #define M_V                                   50.1566f
  #define M_H                                   18.8467f
  #define MAG_DECLINIATION                     -10.1406f
#endif

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                       Error Checking Section                        ***************/
/*************                                                                     ***************/
/*************************************************************************************************/
#ifndef USFS_MAX
  #error "Is this board a USFSMAX?"
#endif

#endif // def_h
