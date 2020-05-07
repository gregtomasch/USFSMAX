# USFSMAX Teensy 3.x Test Sketch

This interconnect information is for operating the [USFSMAX motion coprocessor board](https://cdn.tindiemedia.com/images/resize/S4Os6lUdoJOFuyZHntYHBDiDCUk=/p/full-fit-in/1782x1336/i/44691/products/2020-02-03T20%3A51%3A19.878Z-USFSMAX.top.jpg) with the [PJRC "Teensy" family of development boards](https://www.pjrc.com/teensy/). The default configuration of the sketch is set up to use the "Wire" instance of the TWI I2C library and pin "7" as the data ready (DRDY) interrupt. The necessary connections between the two boards are:

## Prototype Breadboard
|USFSMAX Pin|Teensy3.x Pin|
|:---------:|:-----------:|
|   3V3     |     3V3     |
|   GND     |     GND     |
|   SCL     |     19      |
|   SDA     |     18      |
|   INT     |      7      |

## USFS "Piggybacked" onto Development Board
|USFSMAX Pin|Teensy3.x Pin|
|:---------:|:-----------:|
|   3V3     |      21     |
|   GND     |      22     |
|   SCL     |      19     |
|   SDA     |      18     |
|   INT     |      7      |

![alt text](https://user-images.githubusercontent.com/5760946/80746808-52349180-8ad7-11ea-942d-589a6b66462d.JPG)

![alt text](https://user-images.githubusercontent.com/5760946/80746802-51036480-8ad7-11ea-8b71-cef26d159655.JPG)
