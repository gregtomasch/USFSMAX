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

This sketch was tested for both the Teensy 3.2 and 3.6 development boards, both of which are shown below in piggyback configuration.

![alt text](https://user-images.githubusercontent.com/5760946/80746808-52349180-8ad7-11ea-942d-589a6b66462d.JPG)

![alt text](https://user-images.githubusercontent.com/5760946/80746802-51036480-8ad7-11ea-8b71-cef26d159655.JPG)

## Building/Uploading/Running the Test Sketch

The Sketch is built and uploaded using the ["Teensyduino" core for the Arduino IDE](https://www.pjrc.com/teensy/teensyduino.html):
* Install the "Teensyduino" core as directed [on the Teensduino page on the PJRC website](https://www.pjrc.com/teensy/teensyduino.html)
* Download the USFSMAX Simple Host Utility sketch from this repository and open it with the Arduino IDE
* Go into the "Tools->Board:" menu entry of the IDE and select the appropriate Teensy3.x board variant entry
* Plug the Teensy 3.x development board into an available USB port on your PC
* Build/Upload the sketch as you would with any Arduino board
* Once the upload is complete, power cycle the Teensy3./USFSMAX by unplugging and replugging the USB cable
* Open the Arduino serial monitor at 115200 baud. You should see some startup messages and then the calibration data scroll across the serial monitor
* Send a "1" over the serial monitor; you should see AHRS and sensor continuously scrolling across the serial monitor
