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

#ifndef types_h
#define types_h

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                 Enumerators and Structure Variables                 ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

enum axes
{
  EAST = 0,
  NORTH,
  UP
};

enum attitudes
{
  PITCH = 0,
  ROLL,
  YAW
};

typedef struct
{
  uint16_t cal_points;
  uint8_t  Ascale;
  uint8_t  AODR;
  uint8_t  Alpf;
  uint8_t  Ahpf;
  uint8_t  Gscale;
  uint8_t  GODR;
  uint8_t  Glpf;
  uint8_t  Ghpf;
  uint8_t  Mscale;
  uint8_t  MODR;
  uint8_t  Mlpf;
  uint8_t  Mhpf;
  uint8_t  Pscale;
  uint8_t  PODR;
  uint8_t  Plpf;
  uint8_t  Phpf;
  uint8_t  AUX1scale;
  uint8_t  AUX1ODR;
  uint8_t  AUX1lpf;
  uint8_t  AUX1hpf;
  uint8_t  AUX2scale;
  uint8_t  AUX2ODR;
  uint8_t  AUX2lpf;
  uint8_t  AUX2hpf;
  uint8_t  AUX3scale;
  uint8_t  AUX3ODR;
  uint8_t  AUX3lpf;
  uint8_t  AUX3hpf;
  float    m_v;
  float    m_h;
  float    m_dec;
  uint8_t  quat_div;
} CoProcessorConfig_t;

typedef struct
{
  float V[3];                              // Offset vector components in physical units
  float invW[3][3];                        // Inverse calibration matrix
  uint8_t cal_good;                        // Byte to verify valid cal is in EEPROM
} full_adv_cal_t;

#endif // types_h
