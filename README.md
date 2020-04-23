# USFSMAX Motion Coprocessor

## Introduction
Tlera Corporation offers a popular set of attitude and heading reference system (AHRS) boards known as the "Ultimate Sensor Fusion Solution" or USFS. The heart of the USFS is [EM Microelectronic's EM7180](https://www.emmicroelectronic.com/product/sensor-fusion/em7180-sentral) "Sentral" sensor fusion coprocessor, which uses PNICorp's [“SpacePoint<sup>TM</sup>”](https://www.pnicorp.com/mm-module/) adaptive fusion algorithm. With proper sensor calibration, the USFS can readily provide heading accuracy of [~2deg RMS or better](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf).

Many customers have found the USFS to be a good solution for their applications. However, there are two fundamental limitations to the Sentral sensor fusion solution:
* The ~2deg RMS residual heading error level is dominated by uncorrected (sinusoidal) [systematic sensor errors](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf)
* The SpacePoint adaptive algorithm is "Always on". In the case of external magnetic interference, the algorithm can adapt to local corruption of the geomagnetic field... Potentially giving bad heading results that can persist for several minutes after the magnetic interference has been resolved

Closer examination of the Sentral's residual heading error showed it to be remarkably stable as well as systematic. These facts convinced me that an improved sensor calibration method and user control over any adaptive elements in the fusion algorithm would be key areas to improve beyond the ~2deg RMS heading error level provided by the Sentral. 

These were the primary motivating factors behind development of the ["USFSMAX" motion coprocessor](https://www.tindie.com/products/onehorse/max32660-motion-co-processor/), documented in detail on [hackaday.io](https://hackaday.io/project/160283-max32660-motion-co-processor). The actual coprocessor is Maxim Integrated's [MAX32660 Cortex M4F microcontroller](https://www.maximintegrated.com/en/products/microcontrollers/MAX32660.html). The MAX32660 is paired with ST Micro's [LSM6DSM accel/gyro](https://www.st.com/en/mems-and-sensors/lsm6dsm.html), [LIS2MDL magnetometer](https://www.st.com/en/mems-and-sensors/lis2mdl.html) and [LPS22HB barometer](https://www.st.com/en/mems-and-sensors/lps22hb.html) all of which are "Best in breed" contenders among the various MEMS sensors currently available. The [USFSMAX hardware](https://hackaday.io/project/160283-max32660-motion-co-processor/log/171113-final-hardware-design) is an excellent platform for enhanced sensor calibration and fusion algorithm development:
* Lots of horsepower - 96MHz Cortex M4F CPU
* Lots of memory - 256KB flash, 9KB SRAM
* High-quality, stable MEMS sensors
* 1MHz asynchronous I2C slave bus for fast data transfer to the host MCU

## USFSMAX Performance
Over the period of about two years, I have made a great deal of progress in terms of both the fusion and sensor calibration methods. These advances have been incorporated into the USFSMAX motion coprocessor hardware to provide results that are [significantly better than those from the Sentral](https://cdn.hackaday.io/images/7698711574962560703.jpg). ***Calibration and characterization of the first four prototype USFSMAX [showed RMS heading error ranging between 0.25 and 0.35deg](https://cdn.hackaday.io/images/8316721576958074969.jpg).*** Furthermore, the sinusoidal character of the heading error has been largely eiminated, indicating that the vast majority of systematic sensor errors have been effectively corrected. This level of accuracy can be enabling for many user applications in a smaller form factor and at a fraction of the price of [other AHRS solutions](https://www.xsens.com/inertial-sensor-modules).

But perhaps the most important advance is in the area of improved practical performance. Typically, the sensors can be well-calibrated after bench procedures conducted under controlled conditions... But the actual practical performance degrades during real-world usage out in the field. After extensive testing and observation it became clear that degraded heading accuracy is almost entirely driven by residual ["Hard iron" effects](http://www.jewellinstruments.com/3-factors-that-influence-electronic-compass-accuracy/). I have developed an ***in-situ*** dynamic hard iron (DHI) corrector that is capable of measuring and subtracting any hard-iron-like magnetic interference one the USFSMAX has been installed in a test object for use. To demonstrate the DHI corrector's efficacy, a small rare-earth magnet was attached to the USFSMAX's test fixture after the bench calibration was done. The DHI corrector was reset and "Taught" by tumbling the USFSMAX in 3-D. ***[The results](https://cdn.hackaday.io/images/277781576958101753.jpg) show that with the rare earth magnet attached the heading is unusable (RMS heading error 99.4deg) and that the DHI corrector recovered the heading accuracy back to the same level observed immediately after the bench calibration procedure (RMS heading error 0.18deg).***

## Dynamic Hard Iron (DHI) Corrector
The DHI corrector programmed into the USFSMAX's firmware provides a benefit similar to the adaptive nature of the Sentral's SpacePoint<sup>TM</sup> algorithm but with some key differences:
1. The DHI corrector can be enabled or disabled at startup by user command from the host MCU
2. If there is a valid DHI correction in the USFSMAX's EEPROM, it is loaded and used at startup if the DHI corrector is enabled
3. The DHI corrector starts collecting data at startup/reset and then stops once the new hard iron correction estimate is complete
4. The new DHI correction estimate is automatically stored in the USFSMAX's EEPROM upon completion
5. The DHI corrector can be reset at any time by user command from the host MCU. Once reset, any hard iron correction estimate in the EEPROM is invalidated and data collection for a new correction estimate begins
6. When the new hard iron correction estimate is complete, a quality figure-of-merit for the estimate is available as well. The user can then choose to let the estimate stand or reset the corrector and try again

So, the user can decide when to use the DHI corrector and the timespan over which the correction data set is collected is limited. If the user decides the hard iron correction estimate has become stale, the corrector can be reset at will to begin a new estimate. If transient magnetic interference is not a large issue, the USFSMAX will rely on the last saved hard iron correction estimate until a new one is generated. The quality of the latest correction estimate is judged by the [R-squared metric](https://en.wikipedia.org/wiki/Coefficient_of_determination). When the variation in the magnetic data set NOT explained by the hard iron correction estimate tends to zero, R-squared tends to 1.0. For the moment it is up to the user to interpret the R-squared value and either accept the hard iron correction estimate or reject it and reset the corrector.

As a final matter, there are actually two versions of the DHI corrector: 3-D and 2-D. They are both capable of yielding excellent hard iron correction estimates but both are included for different use cases of the USFSMAX. Simply stated, if the test object to which the USFSMAX is attached is small/light and can easily be tumbled in 3-Space, the 3-D corrector is the best choice. If the test object is unwieldy or its motion is largely constrained to the X-Y (horizontal) plane the 2-D corrector is a better choice. Both correctors collect 150 data points at enforced separation before calculating the final correction estimate. When the estimate is complete, the hard iron corrector status bit in the calibration status register toggles true and the R-squared value populates the appropriate data registers. The desired corrector can be chosen at startup by user command from the host MCU. To get the best hard iron correction estimate:
* 3-D corrector: Tumble the USFSMAX (attached to the test object) in 3-Space trying to sample all possible orientations. If done properly, R-square >= 0.95 is quite normal
* 2-D corrector: Rotate the USFSMAX (attached to the test object) in the X-Y (horizontal) plane. Better hard iron correction estimates are obtained when the USFSMAX is within +/- ~5deg of level during horizontal rotation. If rigorously constrained to be level (pitch = roll = ~0) during rotation, R-square >= 0.95 can be expected. If not rigorously level during horizontal rotation, R-squared tends to be smaller. If R-squared >= ~0.75, the hard iron correction estimate is generally still quite good

## Example Host MCU Sketch
This repository contains an example host MCU Arduino sketch to demonstrate basic use of the USFSMAX motion coprocessor. This particular sketch is written for the [Tlera Dragonfly STM32L476 development board](https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/) using the [STM32L core for the Arduino IDE](https://github.com/GrumpyOldPizza/arduino-STM32L4). This code should be largely transportable to other popular Arduino-programmable microcontroller development boards by updating the I2C library and data ready (DRDY) interrupt pin.

The sketch configures the USFSMAX at startup and demonstrates the basic AHRS functions and sensor data acquisition according to the [USFSMAX register map](https://github.com/gregtomasch/USFSMAX/blob/master/USFSMAX_Reg_Map_0.0.pdf) included in this repository. The DHI corrector can be enabled/configured in the "config.h" tab of the sketch to evaluate its operation in 2-D and 3-D modes. The sketch's serial interface supports reset of the DHI corrector selected at startup.

### Sensor Calibrations
All bench calibrations of the accelerometers and magnetometers are stored in the USFSMAX's EEPROM and are not intended to be modified by the user at this time.

The gyroscope biases are calculated at startup and are stored in the USFSMAX's EEPROM. The gyroscope bias calculation routine detects unwanted motion and resets the data buffers if the USFSMAX is bumped during gyroscope calibration. **However, it is a good idea to take care to not disturb the USFS during gyroscope calibration.** Gyroscope calibration can be selected from the sketch's serial interface at any time.

The DHI correction function is activated by defining "ENABLE_DHI_CORRECTOR" as "0x01". The 2-D corrector is selected by defining "USE_2D_DHI_CORRECTOR" as "0x01", otherwise the 3-D corrector is used instead. These definitions along with some simple instructions are located in the "BASIC SETUP" section in the "config.h" tab of the sketch.

If the 3-D DHI corrector is selected and active, data collection for a new hard iron correction estimate begins at startup. Tumble the USFSMAX (attached to the test object) randomly in 3-Space until the "Dynamic Hard Iron Correction Valid" field on the serial interface toggles from "0" to "128". The new value of the R-squared will be displayed as well.

If the 2-D DHI corrector is selected and active, the basic procedure and serial interface response is similar. However, the USFSMAX and test object should be rotated in the horizontal plane (NOT in 3-D) for several revolutions until the corrector completes the hard iron correction estimate. The closer to level the pitch and roll attitude of the USFSMAX is held during 2-D rotation the better the hard iron correction estimate (and the larger R-squared) will be.

### I2C Data Transactions
**The I2C slave address of the USFSMAX is currently set to 0x57.** There are plans to make the I2C slave address user-selectable and this feature should be available soon. An important aspect of the USFSMAX's I2C slave bus is that there is always a finite delay between when the host MCU requests to read data and when that data is available to be read. Consequently, the USFSMAX will work best with host MCU's that support [I2C clock stretching](https://www.i2c-bus.org/clock-stretching/).

It should be noted that the USFSMAX data registers are set up for efficient I2C data transfer by "Burst reading". That is, a starting data register is specified in the I2C transaction and additional data registers are read sequentially for the total number of bytes specified in the transaction. To improve overall I2C read request response time to the host MCU, not all data registers can be the starting register of an I2C read transaction. Data registers that are directly addressable at the beginning of a read transaction are highlighted in yellow in the register map. So, for example, if a user wanted to read just the two Y-axis gyroscope sensor data bytes from registers 0x07 and 0x08, that is not supported. Instead, the read transaction would begin at register 0x05 and be four bytes long to include the two Y-axis gyroscope data bytes desired.

### Magnetic Constants
Your local geomagnetic constants are necessary to achieve the best heading accuracy. The constants in question are:
* Vertical field strength (M_V)
* Horizontal field strength (M_H)
* Magnetic declination (MAG_DECLINATION)

These constants are available from [online calculators](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm) for your latitude and longitude. Once your geomagnetic constants have been looked up:
* Define an entry for your location in the "Magnetic Constantants" section of the "config.h" tab

  Example:
  
      #define SUNNYVALE_CA_USA
  
* Define a new magnetic constants block in the "Magnetic Constants" section of the "def.h" tab

  Example:
  
      #ifdef SUNNYVALE_CA_USA
          #define M_V                                   41.8128f
          #define M_H                                   23.2519f
          #define MAG_DECLINIATION                      13.2197f
      #endif
  
* Comment out all location definitions in the "Magnetic Constants" section of the "config.h" tab except for your own
