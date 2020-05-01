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

#ifndef config_h
#define config_h

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                     SECTION  1 - BASIC SETUP                        ***************/
/*************                                                                     ***************/
/*************************************************************************************************/
    #define MAX32660_SLV_ADDR                  (0x57)               // USFS MAX I2C slave address
    #define INT_PIN                            7                    // USFSMAX breakout board
    #define LED_PIN                            13                   // Teensy LED
    #define POWER_BY_GPIO_PINS                                      // Uncomment to power the USFSMAX using Teensy GPIO pins
    #define USFS_GND                           22                   // Teensy GPIO pin for USFSMAX GND
    #define USFS_VCC                           21                   // Teensy GPIO pin for USFSMAX 3V3

    // Dynamic Hard Iron corrector (Uncomment one only)
    #define ENABLE_DHI_CORRECTOR               0x01
    //#define ENABLE_DHI_CORRECTOR               0x00

    // Dynamic Hard Iron Corrector allgorithm (Uncomment one only)
    //#define USE_2D_DHI_CORRECTOR               0x01                 // Define as "1" to use the 2D HI corrector instead of the 3D corrector
    #define USE_2D_DHI_CORRECTOR               0x00

    #define SERIAL_DEBUG                                            // Uncomment to see the verbose screen update; comment out for spreadsheet or "MotionCal" GUI output
    //#define MOTION_CAL_GUI_ENABLED                                  // Uncomment to visualize the magnetometer response surface on the "MotionCal" GUI (https://www.pjrc.com/store/prop_shield.html)
    #define UPDATE_PERIOD                      100                  // Serial update period (ms)
    #define CAL_POINTS                         2048                 // Number or data points collected for gyro and accel/fine mag calibrations
    #define AHRS_AVERAGING_POINTS              300                  // Number of screen updates to be averaged for AHRS summary data

    #define SENSOR_0_WIRE_INSTANCE             Wire                 // I2C Wire Library instance

    // Wire pins (Uncomment one only)
    //#define I2C_PINS                           I2C_PINS_16_17
    #define I2C_PINS                           I2C_PINS_18_19

    // I2C Clock Speed (Uncomment one only)
    //#define I2C_CLOCK                          I2C_RATE_100         // 100kHz
    //#define I2C_CLOCK                          I2C_RATE_400         // 400kHz
    #define I2C_CLOCK                          I2C_RATE_1000        // 1MHz

/********************                Alternate Fusion Filter                **********************/
    /* Uncomment one only */
    //#define MADGWICK_9DOF
    #define MAHONY_9DOF


/********************                  MAX32660 IMU Board                   **********************/
    /* Add boards as they become available */
    #define USFS_MAX                                                // Pesky Products USFSMAX

    /* Uncomment one only */
    //#define OUTPUT_EULER_ANGLES                 0x01                // Output Euler angles
    #define OUTPUT_EULER_ANGLES                 0x00                // Output Quaternions


    #define SCALED_SENSOR_DATA                  0x01                // CoPro sensor data is scaled/calibrated/oriented
    //#define SCALED_SENSOR_DATA                  0x00                // CoPro sensor data is raw/not oriented

/**************************            Sensor Data Rates              ****************************/
    /* LSM6DSM Acc Output data rate. Uncomment only one option */
    //#define ACC_ODR                            0x0A                 // 6660Hz
    //#define ACC_ODR                            0x09                 // 3330Hz
    //#define ACC_ODR                            0x08                 // 1660Hz
    #define ACC_ODR                            0x07                 // 834Hz
    //#define ACC_ODR                            0x06                 // 416Hz
    //#define ACC_ODR                            0x05                 // 208Hz
    //#define ACC_ODR                            0x04                 // 104Hz
    //#define ACC_ODR                            0x03                 // 52Hz
    //#define ACC_ODR                            0x02                 // 26Hz
    //#define ACC_ODR                            0x01                 // 12.5Hz

    /* LSM6DSM Gyro Output data rate. Uncomment only one option */
    //#define GYRO_ODR                           0x0A                 // 6660Hz
    //#define GYRO_ODR                           0x09                 // 3330Hz
    //#define GYRO_ODR                           0x08                 // 1660Hz
    #define GYRO_ODR                           0x07                 // 834Hz
    //#define GYRO_ODR                           0x06                 // 416Hz
    //#define GYRO_ODR                           0x05                 // 208Hz
    //#define GYRO_ODR                           0x04                 // 104Hz
    //#define GYRO_ODR                           0x03                 // 52Hz
    //#define GYRO_ODR                           0x02                 // 26Hz
    //#define GYRO_ODR                           0x01                 // 12.5Hz

    /* LIS2MDL Mag Output data rate. Uncomment only one option */
    #define MAG_ODR                            0x03                 // 100Hz
    //#define MAG_ODR                            0x02                 // 50Hz
    //#define MAG_ODR                            0x01                 // 20Hz
    //#define MAG_ODR                            0x00                 // 10Hz

    /* BMP280 Baro Output data rate. Uncomment only one option */
    //#define BARO_ODR                           0x05                 // 75Hz
     #define BARO_ODR                           0x04                 // 50Hz
    //#define BARO_ODR                           0x03                 // 25Hz
    //#define BARO_ODR                           0x02                 // 10Hz
    //#define BARO_ODR                           0x01                 // 1Hz

    /* AUX1 Output data rate. Uncomment only one option */
    #define AUX1_ODR                            0x00                 // Future option

    /* AUX2 Output data rate. Uncomment only one option */
    #define AUX2_ODR                            0x00                 // Future option

    /* AUX2 Output data rate. Uncomment only one option */
    #define AUX3_ODR                            0x00                 // Future option

    /* Quaternion rate divisor; quaternion rate is the gyro ODR (in Hz) divided by the quaternion rate divisor. Uncomment only one option */
    //#define QUAT_DIV                           0x0F                 // 16
    //#define QUAT_DIV                           0x09                 // 10
    #define QUAT_DIV                           0x07                 // 8
    //#define QUAT_DIV                           0x05                 // 6
    //#define QUAT_DIV                           0x04                 // 5
    //#define QUAT_DIV                           0x03                 // 4
    //#define QUAT_DIV                           0x02                 // 3
    //#define QUAT_DIV                           0x01                 // 2
    //#define QUAT_DIV                           0x00                 // 1

/**************************              Sensor Scales                ****************************/
    /* LSM6DSM Acc Output Scale. Uncomment only one option */
    //#define ACC_SCALE_2                                             // +/-2g
    //#define ACC_SCALE_4                                             // +/-4g
    //#define ACC_SCALE_8                                             // +/-8g
    #define ACC_SCALE_16                                            // +/-16g

    /* LSM6DSM Gyro Output Scale. Uncomment only one option */
    //#define GYRO_SCALE_125                                          // +/-125DPS
    //#define GYRO_SCALE_250                                          // +/-250DPS
    //#define GYRO_SCALE_500                                          // +/-500DPS
    //#define GYRO_SCALE_1000                                         // +/-1000DPS
    #define GYRO_SCALE_2000                                         // +/-2000DPS

    /* LSM6DSM Acc Output Scale. Uncomment only one option */
    #define MAG_SCALE                            0x00               // Not adjustable
    
    /* LSM6DSM Acc Output Scale. Uncomment only one option */
    #define BARO_SCALE                           0x00               // Not adjustable

    /* AUX1 Output Scale. Uncomment only one option */
    #define AUX1_SCALE                           0x00               // Future option

    /* AUX2 Output Scale. Uncomment only one option */
    #define AUX2_SCALE                           0x00               // Future option

    /* AUX3 Output Scale. Uncomment only one option */
    #define AUX3_SCALE                           0x00               // Future option

/**************************              Sensor Filters             ****************************/
    /* LSM6DSM Gyro low pass filter setting. Uncomment only one option */
    #define LSM6DSM_GYRO_LPF_167
    //#define LSM6DSM_GYRO_LPF_223
    //#define LSM6DSM_GYRO_LPF_314
    //#define LSM6DSM_GYRO_LPF_655
    
    /* LSM6DSM Gyro high pass filter setting. Uncomment only one option */
    #define LSM6DSM_GYRO_DHPF_CFG              0x00                 // Future option

    /* LSM6DSM Acc low pass filter setting. Uncomment only one option */
    //#define LSM6DSM_ACC_LPF_ODR_DIV2
    //#define LSM6DSM_ACC_LPF_ODR_DIV4
    //#define LSM6DSM_ACC_LPF_ODR_DIV9
    //#define LSM6DSM_ACC_LPF_ODR_DIV50
    //#define LSM6DSM_ACC_LPF_ODR_DIV100
    #define LSM6DSM_ACC_LPF_ODR_DIV400

    /* LSM6DSM Acc high pass filter setting. Uncomment only one option */
    #define LSM6DSM_ACC_DHPF_CFG               0x00                 // Future option

    /* LIS2MDL Mag low pass filter setting. Uncomment only one option */
    //#define LIS2MDL_MAG_LPF                    0x00                 // ODR/2
    #define LIS2MDL_MAG_LPF                    0x01                 // ODR/4

    /* LIS2MDL Mag high pass filter setting. Uncomment only one option */
    #define LIS2MDL_MAG_HPF                    0x00                 // Future option

    /* LPS22HB Mag low pass filter setting. Uncomment only one option */
    //#define LPS22HB_BARO_LPF                   0x00                 // ODR/2
    //#define LPS22HB_BARO_LPF                   0x08                 // ODR/9
    #define LPS22HB_BARO_LPF                   0x0C                 // ODR/20

    /* LPS22HB Mag high pass filter setting. Uncomment only one option */
    #define LPS22HB_BARO_HPF                   0x00                 // Future option
    
    /* Aux_1 low pass filter setting. Uncomment only one option */
    #define AUX1_LPF                           0x00                 // Future option

    /* Aux_1 high pass filter setting. Uncomment only one option */
    #define AUX1_HPF                           0x00                 // Future option

    /* Aux_2 low pass filter setting. Uncomment only one option */
    #define AUX2_LPF                           0x00                 // Future option

    /* Aux_2 high pass filter setting. Uncomment only one option */
    #define AUX2_HPF                           0x00                 // Future option

    /* Aux_3 low pass filter setting. Uncomment only one option */
    #define AUX3_LPF                           0x00                 // Future option

    /* Aux_3 high pass filter setting. Uncomment only one option */
    #define AUX3_HPF                           0x00                 // Future option

/********************                  Magnetic Constants                  ***********************/

      #define KELSEYVILLE_CA_USA
      //#define DANVILLE_CA_USA
      //#define YUBA_CITY_CA_USA
      //#define SUNNYVALE_CA_USA
      //#define MISSISSAUGA_ON_CA
      
/**************************            End Configuration              ****************************/

#endif
