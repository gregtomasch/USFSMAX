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
#include "USFSMAX.h"

USFSMAX::USFSMAX(I2Cdev* i2c, uint8_t sensornum)
{
  _i2c = i2c;
  _sensornum = sensornum;
}

void USFSMAX::init_USFSMAX()
{
  uint8_t STAT;
  uint8_t ConfigByte;
  
  #ifdef SERIAL_DEBUG
    Serial.print("USFSMAX_"); 
    Serial.print(_sensornum);
    Serial.println(":");
    Serial.println("");
  #endif
  #ifdef SERIAL_DEBUG
    Serial.print("Configuring the coprocessor...");
    Serial.println("");
  #endif

  STAT = _i2c->readByte(MAX32660_SLV_ADDR, FUSION_STATUS);                                                                         // Read the coprocessor's current fusion status
  delay(100);
  
  #ifdef SERIAL_DEBUG
    Serial.println("");
    Serial.print("Fusion status: "); Serial.println(STAT);
  #endif
  if(STAT == 0)
  {
    _i2c->writeByte(MAX32660_SLV_ADDR, FUSION_START_STOP, 0x00);                                                                   // Stop sensor fusion
    delay(100);

    // Upload configuration structure variable
    USFSMAX::Upload_cfg(Cfg[_sensornum]);

    // Re-start sensor fusion
    ConfigByte = ((0x01 | EulerQuatFlag << 1) | ScaledSensorDataFlag << 2);                                                        // Set bit0 to re-start fusion; adjust bit1, bit2 for desired output options
    _i2c->writeByte(MAX32660_SLV_ADDR, FUSION_START_STOP, ConfigByte);
    delay(100);

    // Poll the FUSION_STATUS register to see if fusion has resumed
    while(1)
    {
      delay(10);
      STAT = _i2c->readByte(MAX32660_SLV_ADDR, FUSION_STATUS);
      if((STAT & FUSION_RUNNING_MASK)) {break;}
    }
    #ifdef SERIAL_DEBUG
      Serial.println("");
      Serial.println("USFSMAX sensor fusion running!");
      Serial.println("");
    #endif
  }

  // Check for sensor errors
  STAT = _i2c->readByte(MAX32660_SLV_ADDR, SENS_ERR_STAT);
  #ifdef SERIAL_DEBUG
    Serial.println("");
    Serial.print("USFSMAX Sensor Status: "); Serial.print(STAT); Serial.print(" (Should be 0)");
    Serial.println(""); Serial.println("");
  #endif
  if(STAT !=0)
  {
    #ifdef SERIAL_DEBUG
      Serial.print("Sensor error!");
      Serial.println("");
    #endif
    while(1) {;}
  }
  if(ENABLE_DHI_CORRECTOR)
  {
    if(USE_2D_DHI_CORRECTOR)
    {
      _i2c->writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x50);                                                               // Enable DHI corrector, 2D (0x10|0x50)
    } else
    {
      _i2c->writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x10);                                                               // Enable DHI corrector, 3D (0x10)
    }
  }
  Alarms::blink_blueLED(12,100,1);
  delay(100);
  #ifdef SERIAL_DEBUG
    Serial.print("Coprocessor configured! Reading sensor calibrations...");
    Serial.println("");
  #endif
  
  USFSMAX::Retreive_full_gyrocal();
  delay(100);
  Alarms::blink_blueLED(2,10,1);
  USFSMAX::Retreive_full_accelcal();
  delay(100);
  Alarms::blink_blueLED(2,10,1);
  USFSMAX::Retreive_ellip_magcal();
  delay(100);
  Alarms::blink_blueLED(2,10,1);
  USFSMAX::Retreive_final_magcal();
  delay(500);
  Alarms::blink_blueLED(2,100,4);
  #ifdef SERIAL_DEBUG
    Serial.println("");Serial.println("");
    Serial.println("Gyroscope Sensor Offsets (g)");
    Serial.println(gyrocal[_sensornum].V[0], 4);
    Serial.println(gyrocal[_sensornum].V[1], 4);
    Serial.println(gyrocal[_sensornum].V[2], 4); 
    Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[_sensornum].invW[0][0], 4); Serial.print(",");
    Serial.print(gyrocal[_sensornum].invW[0][1], 4); Serial.print(",");
    Serial.println(gyrocal[_sensornum].invW[0][2], 4);
    Serial.print(gyrocal[_sensornum].invW[1][0], 4); Serial.print(",");
    Serial.print(gyrocal[_sensornum].invW[1][1], 4); Serial.print(",");
    Serial.println(gyrocal[_sensornum].invW[1][2], 4);
    Serial.print(gyrocal[_sensornum].invW[2][0], 4); Serial.print(",");
    Serial.print(gyrocal[_sensornum].invW[2][1], 4); Serial.print(",");
    Serial.println(gyrocal[_sensornum].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[_sensornum].V[0], 4);
    Serial.println(accelcal[_sensornum].V[1], 4);
    Serial.println(accelcal[_sensornum].V[2], 4); 
    Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[_sensornum].invW[0][0], 4); Serial.print(",");
    Serial.print(accelcal[_sensornum].invW[0][1], 4); Serial.print(",");
    Serial.println(accelcal[_sensornum].invW[0][2], 4);
    Serial.print(accelcal[_sensornum].invW[1][0], 4); Serial.print(",");
    Serial.print(accelcal[_sensornum].invW[1][1], 4); Serial.print(",");
    Serial.println(accelcal[_sensornum].invW[1][2], 4);
    Serial.print(accelcal[_sensornum].invW[2][0], 4); Serial.print(",");
    Serial.print(accelcal[_sensornum].invW[2][1], 4); Serial.print(",");
    Serial.println(accelcal[_sensornum].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[_sensornum].V[0], 4);
    Serial.println(ellipsoid_magcal[_sensornum].V[1], 4);
    Serial.println(ellipsoid_magcal[_sensornum].V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[_sensornum].invW[0][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[_sensornum].invW[0][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[_sensornum].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[_sensornum].invW[1][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[_sensornum].invW[1][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[_sensornum].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[_sensornum].invW[2][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[_sensornum].invW[2][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[_sensornum].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[_sensornum].V[0], 4);
    Serial.println(final_magcal[_sensornum].V[1], 4);
    Serial.println(final_magcal[_sensornum].V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[_sensornum].invW[0][0], 4); Serial.print(",");
    Serial.print(final_magcal[_sensornum].invW[0][1], 4); Serial.print(",");
    Serial.println(final_magcal[_sensornum].invW[0][2], 4);
    Serial.print(final_magcal[_sensornum].invW[1][0], 4); Serial.print(",");
    Serial.print(final_magcal[_sensornum].invW[1][1], 4); Serial.print(","); 
    Serial.println(final_magcal[_sensornum].invW[1][2], 4);
    Serial.print(final_magcal[_sensornum].invW[2][0], 4); Serial.print(","); 
    Serial.print(final_magcal[_sensornum].invW[2][1], 4); Serial.print(",");
    Serial.println(final_magcal[_sensornum].invW[2][2], 4);
    Serial.println(""); Serial.println("");
  #endif
}

void USFSMAX::Upload_cfg(CoProcessorConfig_t Config)
{
  uint8_t CmdByte;

  CmdByte = 0x08;                                                                                                                  // Clears bit0 to stop fusion an sets bit3 to specify configuration uplaod
  _i2c->writeByte(MAX32660_SLV_ADDR, FUSION_START_STOP, CmdByte);
  delay(1000);
  
  // Assign configuration values
  Config.cal_points        = CAL_POINTS;
  Config.Ascale            = ACC_SCALE;
  Config.AODR              = ACC_ODR;
  Config.Alpf              = LSM6DSM_ACC_DLPF_CFG;
  Config.Ahpf              = LSM6DSM_ACC_DHPF_CFG;
  Config.Gscale            = GYRO_SCALE;
  Config.GODR              = GYRO_ODR;
  Config.Glpf              = LSM6DSM_GYRO_DLPF_CFG;
  Config.Ghpf              = LSM6DSM_GYRO_DHPF_CFG;
  Config.Mscale            = MAG_SCALE;
  Config.MODR              = MAG_ODR;
  Config.Mlpf              = LIS2MDL_MAG_LPF;
  Config.Mhpf              = LIS2MDL_MAG_HPF;
  Config.Pscale            = BARO_SCALE;
  Config.PODR              = BARO_ODR;
  Config.Plpf              = LPS22HB_BARO_LPF;
  Config.Phpf              = LPS22HB_BARO_HPF;
  Config.AUX1scale         = AUX1_SCALE;
  Config.AUX1ODR           = AUX1_ODR;
  Config.AUX1lpf           = AUX1_LPF;
  Config.AUX1hpf           = AUX1_HPF;
  Config.AUX2scale         = AUX2_SCALE;
  Config.AUX2ODR           = AUX2_ODR;
  Config.AUX2lpf           = AUX2_LPF;
  Config.AUX2hpf           = AUX2_HPF;
  Config.AUX3scale         = AUX3_SCALE;
  Config.AUX3ODR           = AUX3_ODR;
  Config.AUX3lpf           = AUX3_LPF;
  Config.AUX3hpf           = AUX3_HPF;
  Config.m_v               = M_V;
  Config.m_h               = M_H;
  Config.m_dec             = MAG_DECLINIATION;
  Config.quat_div          = QUAT_DIV;

  // Assign config structure to byte array upload
  memcpy(cfg_buff, &Config, sizeof(CoProcessorConfig_t));

  // Upload configuration bytes
  _i2c->writeBytes(MAX32660_SLV_ADDR, COPRO_CFG_DATA0, 30, &cfg_buff[0]);
  delay(100);
  _i2c->writeBytes(MAX32660_SLV_ADDR, COPRO_CFG_DATA1, (sizeof(CoProcessorConfig_t) - 30), &cfg_buff[30]);
  delay(100);
}

void USFSMAX::GyroAccelMagBaro_getADC()
{
  uint8_t bytes[21];

  _i2c->readBytes(MAX32660_SLV_ADDR, G_X_L, 21, bytes);
  gyroADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  gyroADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  gyroADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
  accADC[_sensornum][0]  = ((int16_t)bytes[7] << 8) | bytes[6];
  accADC[_sensornum][1]  = ((int16_t)bytes[9] << 8) | bytes[8];
  accADC[_sensornum][2]  = ((int16_t)bytes[11] << 8) | bytes[10];
  magADC[_sensornum][0]  = ((int16_t)bytes[13] << 8) | bytes[12];
  magADC[_sensornum][1]  = ((int16_t)bytes[15] << 8) | bytes[14];
  magADC[_sensornum][2]  = ((int16_t)bytes[17] << 8) | bytes[16];
  baroADC[_sensornum]    = (int32_t)bytes[20] << 16 | (int32_t)bytes[19] << 8 | bytes[18];
}

void USFSMAX::GyroAccel_getADC()
{
  uint8_t bytes[12];

  _i2c->readBytes(MAX32660_SLV_ADDR, G_X_L, 12, bytes);
  gyroADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  gyroADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  gyroADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
  accADC[_sensornum][0]  = ((int16_t)bytes[7] << 8) | bytes[6];
  accADC[_sensornum][1]  = ((int16_t)bytes[9] << 8) | bytes[8];
  accADC[_sensornum][2]  = ((int16_t)bytes[11] << 8) | bytes[10];
}

void USFSMAX::MagBaro_getADC()
{
  uint8_t bytes[9];

  _i2c->readBytes(MAX32660_SLV_ADDR, M_X_L, 9, bytes);
  magADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  magADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  magADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
  baroADC[_sensornum]   = (int32_t)bytes[8] << 16 | (int32_t)bytes[7] << 8 | bytes[6];
}

void USFSMAX::Gyro_getADC()
{
  uint8_t bytes[6];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, G_X_L, 6, bytes);
  gyroADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  gyroADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  gyroADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

void USFSMAX::ACC_getADC()
{
  uint8_t bytes[6];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, A_X_L, 6, bytes);
  accADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  accADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  accADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

void USFSMAX::MAG_getADC()
{
  uint8_t bytes[6];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, M_X_L, 6, bytes);
  magADC[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  magADC[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  magADC[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

void USFSMAX::GetMxMy()
{
  uint8_t bytes[4];
  int16_t H[2];

  _i2c->readBytes(MAX32660_SLV_ADDR, MX_L, 4, bytes);

  H[0] = ((int16_t)bytes[1] << 8) | bytes[0];
  H[1] = ((int16_t)bytes[3] << 8) | bytes[2];

  Mx[_sensornum] = ((float)H[0])*UT_per_Count;
  My[_sensornum] = ((float)H[1])*UT_per_Count;
}

void USFSMAX::getQUAT()
{
  uint8_t bytes[16];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, Q0_BYTE0, 16, bytes);
  qt[_sensornum][0] = uint32_reg_to_float (&bytes[0]);
  qt[_sensornum][1] = uint32_reg_to_float (&bytes[4]);
  qt[_sensornum][2] = uint32_reg_to_float (&bytes[8]);
  qt[_sensornum][3] = uint32_reg_to_float (&bytes[12]);
}

void USFSMAX::getQUAT_Lin()
{
  uint8_t bytes[28];

  _i2c->readBytes(MAX32660_SLV_ADDR, Q0_BYTE0, 28, bytes);
  qt[_sensornum][0] = uint32_reg_to_float (&bytes[0]);
  qt[_sensornum][1] = uint32_reg_to_float (&bytes[4]);
  qt[_sensornum][2] = uint32_reg_to_float (&bytes[8]);
  qt[_sensornum][3] = uint32_reg_to_float (&bytes[12]);
  accLIN[_sensornum][0] = ((int16_t)bytes[17] << 8) | bytes[16];
  accLIN[_sensornum][1] = ((int16_t)bytes[19] << 8) | bytes[18];
  accLIN[_sensornum][2] = ((int16_t)bytes[21] << 8) | bytes[20];
  grav[_sensornum][0]   = ((int16_t)bytes[23] << 8) | bytes[22];
  grav[_sensornum][1]   = ((int16_t)bytes[25] << 8) | bytes[24];
  grav[_sensornum][2]   = ((int16_t)bytes[27] << 8) | bytes[26];
}

void USFSMAX::LIN_ACC_getADC()
{
  uint8_t bytes[12];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, LIN_X_L, 12, bytes);
  accLIN[_sensornum][0] = ((int16_t)bytes[1] << 8) | bytes[0];
  accLIN[_sensornum][1] = ((int16_t)bytes[3] << 8) | bytes[2];
  accLIN[_sensornum][2] = ((int16_t)bytes[5] << 8) | bytes[4];
  grav[_sensornum][0]   = ((int16_t)bytes[7] << 8) | bytes[6];
  grav[_sensornum][1]   = ((int16_t)bytes[9] << 8) | bytes[8];
  grav[_sensornum][2]   = ((int16_t)bytes[11] << 8) | bytes[10];
}

void USFSMAX::BARO_getADC()
{
  uint8_t bytes[3];
  
  _i2c->readBytes(MAX32660_SLV_ADDR, BARO_XL, 3, bytes);
  baroADC[_sensornum] = (int32_t)bytes[2] << 16 | (int32_t)bytes[1] << 8 | bytes[0];
}

void USFSMAX::getDHI_Rsq()
{
  uint8_t bytes[2];

  _i2c->readBytes(MAX32660_SLV_ADDR, DHI_RSQ_L, 2, bytes);
  Rsq = ((float)((int16_t)bytes[1] << 8 | bytes[0]))/10000.0f;
}

void USFSMAX::Reset_DHI()
{
  if(USE_2D_DHI_CORRECTOR)
  {
    _i2c->writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x70);                                                                 // Assert DHI Enable=true, DHI Reset=true and 2D Corrector=true (0x40|0x20|0x10)
  } else
  {
    _i2c->writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x30);                                                                 // Assert DHI Enable=true, DHI Reset=true and 2D Corrector=false (0x00|0x20|0x10)
  }
}

void USFSMAX::Retreive_cfg()
{
  _i2c->readBytes(MAX32660_SLV_ADDR, COPRO_CFG_DATA0, 30, &cfg_buff[0]);
  delay(100);
  _i2c->readBytes(MAX32660_SLV_ADDR, COPRO_CFG_DATA1, (sizeof(CoProcessorConfig_t) - 30), &cfg_buff[30]);
  memcpy(&Cfg[_sensornum], cfg_buff, sizeof(CoProcessorConfig_t));
}

void USFSMAX::Retreive_full_accelcal()
{
  _i2c->readBytes(MAX32660_SLV_ADDR, ACCEL_CAL_DATA0, 30, &AccelCal_buff[0]);
  delay(100);
  _i2c->readBytes(MAX32660_SLV_ADDR, ACCEL_CAL_DATA1, (sizeof(full_adv_cal_t) - 30), &AccelCal_buff[30]);
  memcpy(&accelcal[_sensornum], AccelCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::Retreive_ellip_magcal()
{
  _i2c->readBytes(MAX32660_SLV_ADDR, ELLIP_MAG_CAL_DATA0, 30, &EllipMagCal_buff[0]);
  delay(100);
  _i2c->readBytes(MAX32660_SLV_ADDR, ELLIP_MAG_CAL_DATA1, (sizeof(full_adv_cal_t) - 30), &EllipMagCal_buff[30]);
  memcpy(&ellipsoid_magcal[_sensornum], EllipMagCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::Retreive_final_magcal()
{
  _i2c->readBytes(MAX32660_SLV_ADDR, FINE_MAG_CAL_DATA0, 30, &FineMagCal_buff[0]);
  delay(100);
  _i2c->readBytes(MAX32660_SLV_ADDR, FINE_MAG_CAL_DATA1, (sizeof(full_adv_cal_t) - 30), &FineMagCal_buff[30]);
  memcpy(&final_magcal[_sensornum], FineMagCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::Retreive_full_gyrocal()
{
  _i2c->readBytes(MAX32660_SLV_ADDR, GYRO_CAL_DATA0, 30, &GyroCal_buff[0]);
  delay(100);
  _i2c->readBytes(MAX32660_SLV_ADDR, GYRO_CAL_DATA1, (sizeof(full_adv_cal_t) - 30), &GyroCal_buff[30]);
  memcpy(&gyrocal[_sensornum], GyroCal_buff, sizeof(full_adv_cal_t));
}

float USFSMAX::uint32_reg_to_float (uint8_t *buf)
{
  union
  {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 = (((uint32_t)buf[0]) +
           (((uint32_t)buf[1]) <<  8) +
           (((uint32_t)buf[2]) << 16) +
           (((uint32_t)buf[3]) << 24));
  return u.f;
}
