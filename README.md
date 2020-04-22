# USFSMAX Motion Coprocessor

## Introduction
Tlera Corporation offers a popular set of attitude and heading reference system (AHRS) boards known as the "Ultimate Sensor Fusion Solution" or USFS. The heart of the USFS is [EM Microelectronic's EM7180](https://www.emmicroelectronic.com/product/sensor-fusion/em7180-sentral) "Sentral" sensor fusion coprocessor, which uses PNICorp's [“SpacePoint<sup>TM</sup>”](https://www.pnicorp.com/mm-module/) adaptive fusion algorithm. With proper sensor calibration, the USFS can readily provide heading accuracy of [~2deg RMS or better](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf).

Many customers have found the USFS to be a good solution for their applications. However, there are two fundamental limitations to the Sentral sensor fusion solution:
* The ~2deg RMS residual heading error level is due to uncorrected (sinusoidal) [systematic sensor errors](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf)
* The SpacePoint adaptive algorithm is "Always on". In the case of external magnetic interference, the algorithm can adapt to local corruption of the geomagnetic field... Potentially giving bad heading results that can persist for several minutes after the magnetic interference has been resolved

Closer examination of the Sentral's residual heading error showed it to be remarkably stable as well as systematic. All of these facts convinced me that an improved sensor calibration method and user control over any adaptive elements in the fusion algorithm would be key areas to improve beyond the 2deg RMS heading error plateau. 

These were the primary motivating factors behind development of the "USFSMAX" motion coprocessor, documented in detail on [hackaday.io](https://hackaday.io/project/160283-max32660-motion-co-processor). The actual coprocessor is Maxim Integrated's [MAX32660 Cortex M4F microcontroller](https://www.maximintegrated.com/en/products/microcontrollers/MAX32660.html). The MAX32660 is paired with ST Micro's [LSM6DSM accel/gyro](https://www.st.com/en/mems-and-sensors/lsm6dsm.html), [LIS2MDL magnetometer](https://www.st.com/en/mems-and-sensors/lis2mdl.html) and [LPS22HB barometer](https://www.st.com/en/mems-and-sensors/lps22hb.html) all of which are "Best in breed" contenders among the various MEMS sensors currently available. The [USFSMAX hardware](https://hackaday.io/project/160283-max32660-motion-co-processor/log/171113-final-hardware-design) is an excellent platform for enhanced sensor calibration and fusion algorithm development:
* Lots of horsepower - 96MHz Cortex M4F CPU
* Lots of memory - 256KB flash, 9KB SRAM
* High-quality, stable MEMS sensors

## USFSMAX Performance
Over the period of about two years, I have made a great deal of progress in terms of both the fusion and sensor calibration methods. These have been successfully incorporated into the USFSMAX motion coprocessor hardware to provide results that are [significantly better than those from the Sentral](https://cdn.hackaday.io/images/7698711574962560703.jpg). ***Calibration and characterization of the first four prototype USFSMAX [showed RMS heading error ranging between 0.25 and 0.35deg](https://cdn.hackaday.io/images/8316721576958074969.jpg).*** Furthermore, the sinusoidal character of the heading error has been largely eiminated, indicating that the vast majority of systematic sensor errors have been effectively eliminated.

But perhaps the most important advance is in the area of improved practical performance. Typically, the sensors can be well-calibrated after bench procedures conducted under controlled conditions... But the actual practical performance degrades during real-world usage out in the field. After extensive testing and experimentation it became clear that degraded heading accuracy is almost entirely driven by residual ["Hard iron" effects](http://www.jewellinstruments.com/3-factors-that-influence-electronic-compass-accuracy/). I have developed an ***in-situ*** dynamic hard iron (DHI) corrector that is capable of measuring and subtracting any hard-iron-like magnetic interference one the USFSMAX has been installed for use. To demonstrate the DHI corrector's efficacy, a small rare-earth magnet was attached to the USFSMAX's test fixture after the bench calibration was done. The DHI corrector was reset and "Taught" by tumbling the USFSMAX in 3-D. ***[The results](https://cdn.hackaday.io/images/277781576958101753.jpg) show that with the rare earth magnet attached the heading is unusable (RMS heading error 99.4deg) and that the DHI corrector recovered the heading accuracy back to the same level observed immediately after the bench calibration procedure (RMS heading error 0.18deg).***

## Dynamic Hard Iron (DHI) Corrector
The DHI corrector programmed into the USFSMAX's firmware provides a benefit similar to the adaptive nature of the Sentral's SpacePoint<sup>TM</sup> algorithm but with a few key differences:
1. The DHI corrector can be enabled or disabled at startup by user command from the host MCU
2. If there is a valid DHI correction in the USFSMAX's EEPROM, it is loaded and used at startup if the DHI corrector is enabled
3. The DHI corrector starts collecting data at startup/reset and then stops once the new hard iron correction estimate is complete
4. The new DHI correction estimate is automatically stored in the USFSMAX's EEPROM upon completion
5. The DHI corrector can be reset at any time by user command from the host MCU. Once reset, any hard iron correction estimate in the EEPROM is invalidated and data collection for a new correction estimate begins

