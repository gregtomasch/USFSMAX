# USFSMAX Motion Coprocessor

## Introduction
Tlera Corporation offers a popular set of attitude and heading reference system (AHRS) boards known as the "Ultimate Sensor Fusion Solution" or USFS. The heart of the USFS is [EM Microelectronic's EM7180](https://www.emmicroelectronic.com/product/sensor-fusion/em7180-sentral) "Sentral" sensor fusion coprocessor, which uses PNICorp's [“SpacePoint<sup>TM</sup>”](https://www.pnicorp.com/mm-module/) adaptive fusion algorithm. With proper sensor calibration, the USFS can readily provide heading accuracy of [~2deg RMS or better](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf).

Many customers have found the USFS to be a good solution for their particular applications. However, there are two fundamental limitations to the Sentral sensor fusion solution:
* The ~2deg RMS residual heading error level is due to uncorrected (sinusoidal) [systematic sensor errors](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf)
* The SpacePoint adaptive algorithm is "Always on". In the case of external magnetic interference, the algorithm can adapt to local corruption of the geomagnetic field... Potentially giving bad heading results that can persist for several minutes after the magnetic interference has been resolved

Closer examination of the Sentral's residual heading error showed it to be remarkably stable as well as systematic. All of these facts convinced me that an improved sensor calibration method and user control over any adaptive elements in the fusion algorithm would be key areas to improve beyond the 2deg RMS heading error plateau. 

These were the primary motivating factors behind development of the "USFSMAX" motion coprocessor, documented in detail on [hackaday.io](https://hackaday.io/project/160283-max32660-motion-co-processor). The actual coprocessor is Maxim Integrated's [MAX32660 Cortex M4F microcontroller](https://www.maximintegrated.com/en/products/microcontrollers/MAX32660.html). The MAX32660 is paired with ST Micro's [LSM6DSM accel/gyro](https://www.st.com/en/mems-and-sensors/lsm6dsm.html), [LIS2MDL magnetometer](https://www.st.com/en/mems-and-sensors/lis2mdl.html) and [LPS22HB barometer](https://www.st.com/en/mems-and-sensors/lps22hb.html) all of which are "Best in breed" contenders among the various MEMS sensors currently available. The [USFSMAX hardware](https://hackaday.io/project/160283-max32660-motion-co-processor/log/171113-final-hardware-design) is an excellent platform for enhanced sensor calibration and fusion algorithm development:
* Lots of horsepower - 96MHz Cortex M4F CPU
* Lots of memory - 256KB flash, 9KB SRAM
* High-quality, stable MEMS sensors

