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
#include "IMU.h"

IMU::IMU(USFSMAX* usfsmax, uint8_t sensornum)
{
  _usfsmax   = usfsmax;
  _sensornum = sensornum;
}

/**
* @fn: computeIMU()
*
* @brief: Calculates state estimate using USFSMAX-generated data
* 
* @params: 
* @returns: 
*/
void IMU::computeIMU()
{
  float yaw[2];
  static float buff_roll[2] = {0.0f, 0.0f}, buff_pitch[2] = {0.0f, 0.0f}, buff_heading[2] = {0.0f, 0.0f};

  Begin = micros();
  _usfsmax->getQUAT_Lin();
  Acq_time += micros() - Begin;

  // MAXUSFS Quaternion is ENU
  buff_heading[_sensornum] = atan2f(2.0f*(qt[_sensornum][1]*qt[_sensornum][2] - qt[_sensornum][0]*qt[_sensornum][3]), qt[_sensornum][0]*qt[_sensornum][0] -
                                    qt[_sensornum][1]*qt[_sensornum][1] + qt[_sensornum][2]*qt[_sensornum][2] - qt[_sensornum][3]*qt[_sensornum][3]);
  buff_pitch[_sensornum]   = asinf(2.0f*(qt[_sensornum][2]*qt[_sensornum][3] + qt[_sensornum][0]*qt[_sensornum][1]));
  buff_roll[_sensornum]    = atan2f(2.0f*(qt[_sensornum][0]*qt[_sensornum][2] - qt[_sensornum][1]*qt[_sensornum][3]), qt[_sensornum][0]*qt[_sensornum][0] -
                                    qt[_sensornum][1]*qt[_sensornum][1] - qt[_sensornum][2]*qt[_sensornum][2] + qt[_sensornum][3]*qt[_sensornum][3]);
  buff_heading[_sensornum] *= 57.2957795f;
  buff_pitch[_sensornum]   *= 57.2957795f;
  buff_roll[_sensornum]    *= 57.2957795f;
  angle[_sensornum][0]     = buff_roll[_sensornum];
  angle[_sensornum][1]     = buff_pitch[_sensornum];
  yaw[_sensornum]          = buff_heading[_sensornum];
  heading[_sensornum]      = yaw[_sensornum];                                                                                                                    // Mag declination added in USFSMAX
  if(heading[_sensornum] < 0.0f) heading[_sensornum] += 360.0f;                                                                                                  // Convert heading to 0 - 360deg range
  TimeStamp               = ((float)micros() - (float)Start_time)/1000000.0f;
}

/**
* @fn: compute_Alternate_IMU()
*
* @brief: Calculates orientation estimate using open-source 9DOF quaternion filters
* 
* @params: 
* @returns: 
*/

void IMU::compute_Alternate_IMU()
{
  static float    ax[2], ay[2], az[2], gx[2], gy[2], gz[2], mx[2], my[2], mz[2];
  static float    deltat[2];
  uint32_t        tnow[2];
  static float    rps_per_dps  = RPS_PER_DPS;
  static uint32_t tprev[2]     = {Start_time, Start_time};
  static uint32_t intdeltat[2] = {0, 0};
  
  // Scale quaternion filter inputs to physical units
  ax[_sensornum] = accData[_sensornum][0];                                                                                                                       // In g's
  ay[_sensornum] = accData[_sensornum][1];                                                                                                                       // In g's
  az[_sensornum] = accData[_sensornum][2];                                                                                                                       // In g's
  gx[_sensornum] = gyroData[_sensornum][0]*rps_per_dps;                                                                                                          // In rad/s
  gy[_sensornum] = gyroData[_sensornum][1]*rps_per_dps;                                                                                                          // In rad/s
  gz[_sensornum] = -gyroData[_sensornum][2]*rps_per_dps;                                                                                                         // In rad/s
  mx[_sensornum] = magData[_sensornum][1]*10.0f;                                                                                                                 // In milligauss
  my[_sensornum] = magData[_sensornum][0]*10.0f;                                                                                                                 // In milligauss
  mz[_sensornum] = -magData[_sensornum][2]*10.0f;                                                                                                                // In milligauss

  // Calculate deltat for the quaternion update
  tnow[_sensornum]      = micros();
  intdeltat[_sensornum] = tnow[_sensornum] - tprev[_sensornum];
  deltat[_sensornum]    = (float)intdeltat[_sensornum]/1000000.0f;                                                                                               // Result is in seconds
  tprev[_sensornum]     = tnow[_sensornum];
  #ifdef MADGWICK_9DOF
    IMU::MadgwickQuaternionUpdate(ax[_sensornum], ay[_sensornum], az[_sensornum], gx[_sensornum], gy[_sensornum], gz[_sensornum],
                                  mx[_sensornum], my[_sensornum], mz[_sensornum],deltat[_sensornum], QT[_sensornum]);                                            // Alternative quaternion filter
    for(uint8_t i=0; i<10; i++)
    {
      IMU::MadgwickQuaternionUpdate(ax[_sensornum], ay[_sensornum], az[_sensornum], 0.0f, 0.0f, 0.0f,
                                    mx[_sensornum], my[_sensornum], mz[_sensornum],deltat[_sensornum], QT[_sensornum]);                                          // Alternative iteration; angular velocity zeroed-out
    }
  #endif
  #ifdef MAHONY_9DOF
    IMU::MahonyQuaternionUpdate(ax[_sensornum], ay[_sensornum], az[_sensornum], gx[_sensornum], gy[_sensornum], gz[_sensornum],
                                mx[_sensornum], my[_sensornum], mz[_sensornum],deltat[_sensornum], QT[_sensornum]);                                              // Alternative quaternion filter
    for(uint8_t i=0; i<10; i++)
    {
      IMU::MahonyQuaternionUpdate(ax[_sensornum], ay[_sensornum], az[_sensornum], 0.0f, 0.0f, 0.0f,
                                  mx[_sensornum], my[_sensornum], mz[_sensornum],deltat[_sensornum], QT[_sensornum]);                                            // Alternative iteration; angular velocity zeroed-out
    }
  #endif
  HEADING[_sensornum]  = -atan2f(QT[_sensornum][0]*QT[_sensornum][0] - QT[_sensornum][1]*QT[_sensornum][1] + QT[_sensornum][2]*QT[_sensornum][2] -
                                 QT[_sensornum][3]*QT[_sensornum][3], 2.0f*(QT[_sensornum][1]*QT[_sensornum][2] - QT[_sensornum][0]*QT[_sensornum][3]));   
  ANGLE[_sensornum][1] = asinf(2.0f*(QT[_sensornum][2]*QT[_sensornum][3] + QT[_sensornum][0]*QT[_sensornum][1]));
  ANGLE[_sensornum][0] = atan2f(2.0f*(QT[_sensornum][0]*QT[_sensornum][2] - QT[_sensornum][1]*QT[_sensornum][3]), QT[_sensornum][0]*QT[_sensornum][0] -
                                QT[_sensornum][1]*QT[_sensornum][1] - QT[_sensornum][2]*QT[_sensornum][2] + QT[_sensornum][3]*QT[_sensornum][3]);
  ANGLE[_sensornum][0] *= 57.2957795f;
  ANGLE[_sensornum][1] *= 57.2957795f;
  HEADING[_sensornum]  *= 57.2957795f;
  HEADING[_sensornum]  += MAG_DECLINIATION;
  if(HEADING[_sensornum] < 0.0f) {HEADING[_sensornum] += 360.0f;}                                                                                                // Ensure yaw stays between 0 and 360
}

__attribute__((optimize("O3"))) void IMU::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz,
                                                                 float mx, float my, float mz, float deltat, float* quat)
{
  float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];                                                                                                  // Short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q1q4 = q1*q4;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q2q4 = q2*q4;
  float q3q3 = q3*q3;
  float q3q4 = q3*q4;
  float q4q4 = q4*q4;   

  // Normalize accelerometer measurement
  norm = sqrtf(ax*ax + ay*ay + az*az);
  if(norm == 0.0f) return;                                                                                                                                       // Handle NaN
  norm = 1.0f/norm;                                                                                                                                              // Use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx*mx + my*my + mz*mz);
  if(norm == 0.0f) return;                                                                                                                                       // Handle NaN
  norm = 1.0f/norm;                                                                                                                                              // Use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f*mx*(0.5f - q3q3 - q4q4) + 2.0f*my*(q2q3 - q1q4) + 2.0f*mz*(q2q4 + q1q3);
  hy = 2.0f*mx*(q2q3 + q1q4) + 2.0f*my*(0.5f - q2q2 - q4q4) + 2.0f*mz*(q3q4 - q1q2);
  bx = sqrtf((hx*hx) + (hy*hy));
  bz = 2.0f*mx*(q2q4 - q1q3) + 2.0f*my*(q3q4 + q1q2) + 2.0f*mz*(0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f*(q2q4 - q1q3);
  vy = 2.0f*(q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f*bx*(0.5f - q3q3 - q4q4) + 2.0f*bz*(q2q4 - q1q3);
  wy = 2.0f*bx*(q2q3 - q1q4) + 2.0f*bz*(q1q2 + q3q4);
  wz = 2.0f*bx*(q1q3 + q2q4) + 2.0f*bz*(0.5f - q2q2 - q3q3);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
  if (twoKi > 0.0f)
  {
    integralFBx += ex;                                                                                                                                               // Accumulate integral error
    integralFBy += ey;
    integralFBz += ez;
  } else
  {
    integralFBx = 0.0f;                                                                                                                                              // Prevent integral wind up
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Apply feedback terms
  gx = gx + twoKp*ex + twoKi*integralFBx;
  gy = gy + twoKp*ey + twoKi*integralFBy;
  gz = gz + twoKp*ez + twoKi*integralFBz;

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2*gx - q3*gy - q4*gz)*(0.5f*deltat);
  q2 = pa + (q1*gx + pb*gz - pc*gy)*(0.5f*deltat);
  q3 = pb + (q1*gy - pa*gz + pc*gx)*(0.5f*deltat);
  q4 = pc + (q1*gz + pa*gy - pb*gx)*(0.5f*deltat);

  // Normalize quaternion
  norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  norm = 1.0f/norm;
  quat[0] = q1*norm;
  quat[1] = q2*norm;
  quat[2] = q3*norm;
  quat[3] = q4*norm;
}

__attribute__((optimize("O3"))) void IMU::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz,
                                                                   float mx, float my, float mz, float deltat, float* quat)
{
  float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];                                                                                                      // Short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalize accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;                                                                                                                                          // Handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalize magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return;                                                                                                                                          // Handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
       (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + 
       _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
       (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
       (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) +
       _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) +
       _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) +
       _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);                                                                                                               // Normalize step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);                                                                                                               // Normalize quaternion
  norm = 1.0f/norm;
  quat[0] = q1 * norm;
  quat[1] = q2 * norm;
  quat[2] = q3 * norm;
  quat[3] = q4 * norm;
}
