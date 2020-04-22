# USFSMAX Motion Coprocessor

## Introduction
Tlera Corporation offers a popular set of attitude and heading reference system (AHRS) boards known as the "Ultimate Sensor Fusion Solution" or USFS. The heart of the USFS is [EM Microelectronic's EM7180](https://www.emmicroelectronic.com/product/sensor-fusion/em7180-sentral) "Sentral" sensor fusion coprocessor, which uses PNICorp's [“SpacePoint<sup>TM</sup>”](https://www.pnicorp.com/mm-module/) adaptive fusion algorithm. With proper sensor calibration, the USFS can readily provide heading accuracy of [~2deg RMS or better](https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/K.-Limits-of-Absolute-Heading-Accuracy-Using-Inexpensive-MEMS-Sensors).

Many customers have found the USFS to be a good solution for their particular applications. However, there are two fundamental limitations to the Sentral sensor fusion solution:
* The ~2deg RMS residual heading error level is due to uncorrected (sinusoidal) [systematic sensor errors](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf)
* The SpacePoint adaptive algorithm is "Always on". In the case of external magnetic interference, the algorithm can adapt to local corruption of the geomagnetic field... Potentially giving bad heading results that can persist for several minutes after the magnetic interference has been resolved
