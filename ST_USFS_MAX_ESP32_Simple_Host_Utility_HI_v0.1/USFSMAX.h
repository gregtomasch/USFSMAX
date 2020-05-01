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

#ifndef USFSMAX_h
#define USFSMAX_h

#include "I2Cdev.h"
#include "Alarms.h"
#include "def.h"
#include "config.h"
#include "Types.h"

#define SENS_ERR_STAT                 0x00
#define CALIBRATION_STATUS            0x01
#define ACCEL_CAL_POS                 0x02
#define FUSION_STATUS                 0x03
#define COMBO_DRDY_STAT               0x04

#define G_X_L                         0x05
#define G_X_H                         0x06
#define G_Y_L                         0x07
#define G_Y_H                         0x08
#define G_Z_L                         0x09
#define G_Z_H                         0x0A
#define A_X_L                         0x0B
#define A_X_H                         0x0C
#define A_Y_L                         0x0D
#define A_Y_H                         0x0E
#define A_Z_L                         0x0F
#define A_Z_H                         0x10
#define M_X_L                         0x11
#define M_X_H                         0x12
#define M_Y_L                         0x13
#define M_Y_H                         0x14
#define M_Z_L                         0x15
#define M_Z_H                         0x16
#define BARO_XL                       0x17
#define BARO_L                        0x18
#define BARO_H                        0x19
#define Q0_BYTE0                      0x1A
#define Q0_BYTE1                      0x1B
#define Q0_BYTE2                      0x1C
#define Q0_BYTE3                      0x1D
#define Q1_BYTE0                      0x1E
#define Q1_BYTE1                      0x1F
#define Q1_BYTE2                      0x20
#define Q1_BYTE3                      0x21
#define Q2_BYTE0                      0x22
#define Q2_BYTE1                      0x23
#define Q2_BYTE2                      0x24
#define Q2_BYTE3                      0x25
#define Q3_BYTE0                      0x26
#define Q3_BYTE1                      0x27
#define Q3_BYTE2                      0x28
#define Q3_BYTE3                      0x29
#define LIN_X_L                       0x2A
#define LIN_X_H                       0x2B
#define LIN_Y_L                       0x2C
#define LIN_Y_H                       0x2D
#define LIN_Z_L                       0x2E
#define LIN_Z_H                       0x2F
#define GRAV_X_L                      0x30
#define GRAV_X_H                      0x31
#define GRAV_Y_L                      0x32
#define GRAV_Y_H                      0x33
#define GRAV_Z_L                      0x34
#define GRAV_Z_H                      0x35
#define YAW_BYTE0                     0x36
#define YAW_BYTE1                     0x37
#define YAW_BYTE2                     0x38
#define YAW_BYTE3                     0x39
#define PITCH_BYTE0                   0x3A
#define PITCH_BYTE1                   0x3B
#define PITCH_BYTE2                   0x3C
#define PITCH_BYTE3                   0x3D
#define ROLL_BYTE0                    0x3E
#define ROLL_BYTE1                    0x3F
#define ROLL_BYTE2                    0x40
#define ROLL_BYTE3                    0x41
#define AG_TEMP_L                     0x42
#define AG_TEMP_H                     0x43
#define M_TEMP_L                      0x44
#define M_TEMP_H                      0x45
#define B_TEMP_L                      0x46
#define B_TEMP_H                      0x47
#define AUX_1_X_L                     0x48
#define AUX_1_X_H                     0x49
#define AUX_1_Y_L                     0x4A
#define AUX_1_Y_H                     0x4B
#define AUX_1_Z_L                     0x4C
#define AUX_1_Z_H                     0x4D
#define AUX_2_X_L                     0x4E
#define AUX_2_X_H                     0x4F
#define AUX_2_Y_L                     0x50
#define AUX_2_Y_H                     0x51
#define AUX_2_Z_L                     0x52
#define AUX_2_Z_H                     0x53
#define AUX_3_X_L                     0x54
#define AUX_3_X_H                     0x55
#define AUX_3_Y_L                     0x56
#define AUX_3_Y_H                     0x57
#define AUX_3_Z_L                     0x58
#define AUX_3_Z_H                     0x59
#define MX_L                          0x5A
#define MX_H                          0x5B
#define MY_L                          0x5C
#define MY_H                          0x5D
#define DHI_RSQ_L                     0x5E
#define DHI_RSQ_H                     0x5F

#define FUSION_START_STOP             0x60
#define CALIBRATION_REQUEST           0x61

#define COPRO_CFG_DATA0               0x62
#define COPRO_CFG_DATA1               0x63
#define GYRO_CAL_DATA0                0x64
#define GYRO_CAL_DATA1                0x65
#define ACCEL_CAL_DATA0               0x66
#define ACCEL_CAL_DATA1               0x67
#define ELLIP_MAG_CAL_DATA0           0x68
#define ELLIP_MAG_CAL_DATA1           0x69
#define FINE_MAG_CAL_DATA0            0x6A
#define FINE_MAG_CAL_DATA1            0x6B

#define FUSION_RUNNING_MASK           0x01
#define HI_CORRECTOR_MASK             0x10

extern CoProcessorConfig_t            Cfg[2];
extern uint8_t                        cfg_buff[sizeof(CoProcessorConfig_t)];
extern full_adv_cal_t                 gyrocal[2];
extern full_adv_cal_t                 ellipsoid_magcal[2];
extern full_adv_cal_t                 accelcal[2];
extern full_adv_cal_t                 final_magcal[2];
extern uint8_t                        GyroCal_buff[sizeof(full_adv_cal_t)];
extern uint8_t                        EllipMagCal_buff[sizeof(full_adv_cal_t)];
extern uint8_t                        AccelCal_buff[sizeof(full_adv_cal_t)];
extern uint8_t                        FineMagCal_buff[sizeof(full_adv_cal_t)];
extern uint8_t                        EulerQuatFlag;
extern uint8_t                        ScaledSensorDataFlag;
extern int16_t                        gyroADC[2][3];
extern int16_t                        accADC[2][3];
extern int16_t                        magADC[2][3];
extern float                          Mx[2], My[2];
extern float                          UT_per_Count;
extern float                          qt[2][4];
extern int16_t                        accLIN[2][3];
extern int16_t                        grav[2][3];
extern int32_t                        baroADC[2];
extern float                          Rsq;

class USFSMAX
{
  public:
                                      USFSMAX(I2Cdev*, uint8_t);
     void                             init_USFSMAX();
     void                             GyroAccelMagBaro_getADC();
     void                             GyroAccel_getADC();
     void                             MagBaro_getADC();
     void                             Gyro_getADC();
     void                             ACC_getADC();
     void                             MAG_getADC();
     void                             GetMxMy();
     void                             getQUAT();
     void                             LIN_ACC_getADC();
     void                             getQUAT_Lin();
     void                             BARO_getADC();
     void                             Reset_DHI();
     void                             getDHI_Rsq();
     void                             Retreive_cfg();
     void                             Upload_cfg(CoProcessorConfig_t Cfg);
     void                             Retreive_full_accelcal();
     void                             Retreive_ellip_magcal();
     void                             Retreive_final_magcal();
     void                             Retreive_full_gyrocal();
  private:
     I2Cdev*                          _i2c;
     uint8_t                          _sensornum;
     float                            uint32_reg_to_float (uint8_t *buf);
};

#endif // USFSMAX_h
