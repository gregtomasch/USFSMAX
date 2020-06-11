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

#include <i2c_t3.h>
#include "Alarms.h"
#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"

// Instantiate class objects
I2Cdev     i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX    USFSMAX_0(&i2c_0, 0);
IMU        imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions
void       ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void       FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void       DRDY_handler_0();
void       SerialInterface_handler();

void setup()
{
  #ifdef POWER_BY_GPIO_PINS
    pinMode(USFS_GND, OUTPUT);                                                                                       // Set USFSMAX ground pin
    digitalWrite(USFS_GND, LOW);
    pinMode(USFS_VCC, OUTPUT);                                                                                       // Power up the USFSMAX
    digitalWrite(USFS_VCC, HIGH);
    delay(100);
  #endif
  
  // Open serial port
  Serial.begin(115200);
  delay(4000);

  // Set up DRDY interrupt pin
  pinMode(INT_PIN, INPUT);
  
  // Assign Indicator LED
  LEDPIN_PINMODE;
  Alarms::blueLEDoff();
  
  // Initialize USFSMAX_0 I2C bus
  SENSOR_0_WIRE_INSTANCE.begin(I2C_MASTER, 0x00, I2C_PINS, I2C_PULLUP_EXT, I2C_CLOCK);
  delay(100);
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_RATE_100);                                                                     // Set I2C clock speed to 100kHz cor configuration
  delay(2000);

  // Do I2C bus scan if serial debug is active
  #ifdef SERIAL_DEBUG                                                                                                // Should see MAX32660 slave bus address (default is 0x57)
    i2c_0.I2Cscan();                                           
  #endif

  // Initialize USFSMAX_0
  #ifdef SERIAL_DEBUG
    Serial.print("Initializing USFSMAX_0...");
    Serial.println("");
  #endif
  USFSMAX_0.init_USFSMAX();                                                                                          // Configure USFSMAX and sensors 
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                        // Set the I2C clock to high speed for run-mode data collection
  delay(100);

  // Attach interrupts
  attachInterrupt(INT_PIN, DRDY_handler_0, RISING);                                                                  // Attach DRDY interrupt
  #ifdef SERIAL_DEBUG
    Serial.println("USFXMAX_0 successfully initialized!");
    Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the results
  #endif

  // Calculate geomagnetic calibration parameters for your location (set in "config.h")
  Mv_Cal  = M_V;                                                                                                     // Vertical geomagnetic field component
  Mh_Cal  = M_H;                                                                                                     // Horizontal geomagnetic field component
  M_Cal   = sqrt(Mv_Cal*Mv_Cal + Mh_Cal*Mh_Cal);                                                                     // Geomagnetic field strength
  Del_Cal = atan(Mv_Cal/Mh_Cal);                                                                                     // Geomagnetic inclination or "Dip" angle

  #if !defined(SERIAL_DEBUG)  && !defined(MOTION_CAL_GUI_ENABLED)                                                    // Print header for spreadsheet data collection
    Serial.print("Time");        Serial.print(","); Serial.print("Heading (deg)"; Serial.print(",");
    Serial.print("Pitch (deg)"); Serial.print(","); Serial.print("Roll (deg)");   Serial.print(",");
    Serial.print("Cal Status");  Serial.println("");
  #endif

  Start_time = micros();                                                                                             // Set sketch start time
}

void loop()
{
  // Calculate loop cycle time
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  if(data_ready[0] == 1)
  {
    data_ready[0] = 0;
    ProcEventStatus(&i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);                                                                        // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
  }

  // Update serial output
  delt_t = millis() - last_refresh;
  if (delt_t > UPDATE_PERIOD)                                                                                        // Update the serial monitor every "UPDATE_PERIOD" ms
  {
    last_refresh = millis();
    USFSMAX_0.GetMxMy();                                                                                             // Get Horizontal magnetic components
    #ifdef SERIAL_DEBUG
      SerialInterface_handler();
      if(ENABLE_DHI_CORRECTOR)
      {
        cal_status[0] = i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS);                                       // Poll calibration status byte
        USFSMAX_0.getDHI_Rsq();                                                                                      // Get DHI R-square
        Serial.print("Dynamic Hard Iron Correction Valid = ");
        Serial.println(cal_status[0] & 0x80);                                                                        // DHI correction status
        Serial.print("Dynamic Hard Iron Fit R-square = ");
        Serial.println(Rsq, 4);
        if(USE_2D_DHI_CORRECTOR)
        {
          Serial.println("Using the 2D Corrector");
        } else
        {
          Serial.println("Using the 3D Corrector");
        }
        Serial.println("");
      } else
      {
        Serial.print("Dynamic Hard Iron Correction Disabled!");
        Serial.println(""); Serial.println("");
      }

      // USFSMAX_0 sensor and raw quaternion outout
      Serial.print("ax = "); Serial.print((int)(1000.0f*accData[0][0])); Serial.print(" ay = "); Serial.print((int)(1000.0f*accData[0][1]));
      Serial.print(" az = "); Serial.print((int)(1000.0f*accData[0][2])); Serial.println(" mg");
      Serial.print("gx = "); Serial.print(gyroData[0][0], 1); Serial.print(" gy = "); Serial.print(gyroData[0][1], 1); 
      Serial.print(" gz = "); Serial.print(gyroData[0][2], 1); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print(magData[0][0], 1); Serial.print(" my = "); Serial.print(magData[0][1], 1);
      Serial.print(" mz = "); Serial.print(magData[0][2], 1); Serial.println(" uT");
      Serial.print("Tomasch Xh, Yh: ");
      Serial.print(Mx[0], 2); Serial.print(", "); Serial.print(My[0], 2); Serial.println(" uT");
      Serial.print("Baro pressure = "); Serial.print(((float)baroADC[0])/4096.0f); Serial.println(" hPa");
      Serial.println("");
      Serial.print("USFSMAX Quat: "); Serial.print("q0 = "); Serial.print(qt[0][0], 4);
      Serial.print(" qx = "); Serial.print(qt[0][1], 4); Serial.print(" qy = "); Serial.print(qt[0][2], 4); 
      Serial.print(" qz = "); Serial.print(qt[0][3], 4); Serial.println("");

      // Euler angles
      Serial.print("USFSMAX Yaw, Pitch, Roll: ");
      Serial.print(heading[0], 2); Serial.print(", "); Serial.print(angle[0][1], 2); Serial.print(", "); Serial.println(angle[0][0], 2);

      // Critical time deltas
      //Serial.println(""); Serial.print("Loop CT:"); Serial.print(cycleTime); Serial.println(" us");
      Serial.print("Sensor Acq Time:"); Serial.print(Acq_time); Serial.println(" us"); Serial.println("");
    #endif

    // Spreadsheet output when "SERIAL_DEBUG" and "MOTION_CAL_GUI_ENABLED" are not defined in config.h
    #if !defined(SERIAL_DEBUG)  && !defined(MOTION_CAL_GUI_ENABLED)
      Serial.print(TimeStamp, 2);   Serial.print(","); Serial.print(heading[0], 2);  Serial.print(",");
      Serial.print(angle[0][1], 2); Serial.print(","); Serial.print(angle[0][0], 2); Serial.print(",");
      Serial.print(cal_status[0]);  Serial.println("");
    #endif

    // Output formatted MotionCal GUI magnetometer data message when "MOTION_CAL_GUI_ENABLED" is defined and "SERIAL_DEBUG" is not defined in config.h
    // https://www.pjrc.com/store/prop_shield.html
    #if defined(MOTION_CAL_GUI_ENABLED) && !defined(SERIAL_DEBUG)
      Serial.print("Raw:");
      Serial.print(0);                                                                                               // MotionCal GUI doesn't act upon accel/gyro input; send null data
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print((int16_t)(magData[0][0]*10.0f));                                                                  // The MotionCal GUI is expecting 0.1uT/LSB
      Serial.print(',');
      Serial.print((int16_t)(magData[0][1]*10.0f));
      Serial.print(',');
      Serial.print((int16_t)(magData[0][2]*10.0f));
      Serial.println();
    #endif

    // Toggle LED if not calibrating gyroscopes
    if(gyroCalActive[0] == 1)
    {
      Alarms::blueLEDoff();
      if((i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS) & 0x01) == 0)
      {
        gyroCalActive[0] = 0;
      }
    } else
    {
      Alarms::toggle_blueLED();
    }
    data_ready[0] = 0;
  }
}

void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
  uint8_t temp[1];

  // Read algorithm status and event status
  i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
  eventStatus[sensorNUM] = temp[0];

  // Decode the event status to determine what data is ready and set the appropriate DRDY fags
  if(eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}

void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM)
{
  uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

  Acq_time = 0;
  Begin = micros();

  // Optimize the I2C read function with respect to whatever sensor data is ready
  switch(call_sensors)
  {
   case 0x01:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x02:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x03:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x07:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0B:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0F:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0C:
     usfsmax->MagBaro_getADC();
     break;
   case 0x04:
     usfsmax->MAG_getADC();
     break;
   case 0x08:
     usfsmax->BARO_getADC();
     break;
   default:
     break;
  };
  Acq_time += micros() - Begin;

  if(Mag_flag[sensorNUM])
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        magData[sensorNUM][i] = ((float)magADC[sensorNUM][i])*UT_per_Count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
      sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
      MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Mag_flag[sensorNUM] = 0;
  }
  if(Acc_flag[sensorNUM])
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        accData[sensorNUM][i] = ((float)accADC[sensorNUM][i])*g_per_count;
      } 
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
      ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Acc_flag[sensorNUM] = 0;
  }
  if(Gyro_flag[sensorNUM] == 1)
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i])*dps_per_count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
      GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }

    // Call alternative (Madgwick or Mahony) IMU fusion filter
    IMu->compute_Alternate_IMU();
    Gyro_flag[sensorNUM] = 0;
  }
  if(Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
}

// Host DRDY interrupt handler
void DRDY_handler_0()
{
  data_ready[0] = 1;
}

// Serial interface handler
void SerialInterface_handler()
{
  serial_input = 0;
  if(Serial.available()) serial_input = Serial.read();
  if(serial_input == 49) {sensor_cal.GyroCal();}                                                                     // Type "1" to initiate USFSMAX_0 Gyro Cal
  if(serial_input == 50)                                                                                             // Type "2" to list current sensor calibration data
  {
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_RATE_100);                                                                   // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_full_gyrocal();
    delay(100);
    USFSMAX_0.Retreive_full_accelcal();
    delay(100);
    USFSMAX_0.Retreive_ellip_magcal();
    delay(100);
    USFSMAX_0.Retreive_final_magcal();
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);

    // Print the calibration results
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(gyrocal[0].V[0], 4);
    Serial.println(gyrocal[0].V[1], 4);
    Serial.println(gyrocal[0].V[2], 4); Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[0][2], 4);
    Serial.print(gyrocal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[1][2], 4);
    Serial.print(gyrocal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[0].V[0], 4);
    Serial.println(accelcal[0].V[1], 4);
    Serial.println(accelcal[0].V[2], 4); Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[0][2], 4);
    Serial.print(accelcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[1][2], 4);
    Serial.print(accelcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[0].V[0], 4);
    Serial.println(ellipsoid_magcal[0].V[1], 4);
    Serial.println(ellipsoid_magcal[0].V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[0].V[0], 4);
    Serial.println(final_magcal[0].V[1], 4);
    Serial.println(final_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[0][2], 4);
    Serial.print(final_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[1][2], 4);
    Serial.print(final_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  if(serial_input == 51) {USFSMAX_0.Reset_DHI();}                                                                    // Type "3" to reset the DHI corrector
  serial_input = 0;
  
  // Hotkey messaging
  Serial.println("'1' Gyro Cal");
  Serial.println("'2' List Cal Data");
  Serial.println("'3' Reset DHI Corrector");
  Serial.println("");
}
