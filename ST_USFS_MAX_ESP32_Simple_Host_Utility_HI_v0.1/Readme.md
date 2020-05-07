# USFSMAX ESP32 Test Sketch

This interconnect information is for operating the [USFSMAX motion coprocessor board](https://cdn.tindiemedia.com/images/resize/S4Os6lUdoJOFuyZHntYHBDiDCUk=/p/full-fit-in/1782x1336/i/44691/products/2020-02-03T20%3A51%3A19.878Z-USFSMAX.top.jpg) with the Tlera [ESP32 development board](https://www.tindie.com/products/onehorse/smallest-esp32-development-board/). The default configuration of the sketch is set up to use the "Wire" instance of the TWI I2C library and pin "27" as the data ready (DRDY) interrupt. The necessary connections between the two boards are:

## Prototype Breadboard
|USFSMAX Pin|Dragonfly Pin|
|:---------:|:-----------:|
|   3V3     |     3V3     |
|   GND     |     GND     |
|   SCL     |      15     |
|   SDA     |      16     |
|   INT     |      27     |

## USFS "Piggybacked" onto Development Board
|USFSMAX Pin|Dragonfly Pin|
|:---------:|:-----------:|
|   3V3     |      13     |
|   GND     |      12     |
|   SCL     |      15     |
|   SDA     |      16     |
|   INT     |      27     |

![alt text](https://user-images.githubusercontent.com/5760946/80746815-5365be80-8ad7-11ea-8e1b-241ee2c26463.JPG)

## Building/Uploading/Running the Test Sketch

The Sketch is built and uploaded using the [ESP32 core for the Arduino IDE](https://github.com/espressif/arduino-esp32):
* Install the ESP32 core as directed in the repository
* Download the ESP32 USFSMAX Simple Host Utility sketch from this repository and open it with the Arduino IDE
* Go into the "Tools->Board:" menu entry of the IDE and select the "Onehorse ESP32 Dev Module" entry
* Plug the ESP32 development board into an available USB port on your PC
* Build/Upload the sketch as you would with any Arduino board
* Once the upload is complete, power cycle the ESP32/USFSMAX by unplugging and replugging the USB cable
* Open the Arduino serial monitor at 115200 baud. You should see some startup messages and then the calibration data scroll across the serial monitor
* Send a "1" over the serial monitor; you should see AHRS and sensor continuously scrolling across the serial monitor
