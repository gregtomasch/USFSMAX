# USFSMAX Test Sketch

## Pinout/Connections

The interconnect information given here is for operating the [USFSMAX motion coprocessor board](https://cdn.tindiemedia.com/images/resize/S4Os6lUdoJOFuyZHntYHBDiDCUk=/p/full-fit-in/1782x1336/i/44691/products/2020-02-03T20%3A51%3A19.878Z-USFSMAX.top.jpg) with the Dragonfly [STM32L476 development board](https://cdn.tindiemedia.com/images/resize/c3IuiwcFMGzoMgmNkZ_doDvJiuU=/p/full-fit-in/1782x1336/i/32456/products/2017-01-04T23%3A18%3A53.356Z-2016-05-14T16%2031%2059.360Z-Layout.jpg.855x570_q85_pad_rcrop.jpg). The default configuration of the sketch is set up to use the "Wire2" instance of the TWI I2C library and pin "A3" as the data ready (DRDY) interrupt. The necessary connections between the two boards are:

    Dragonfly Pin      USFSMAX Pin
    3V3            ->  3V3
    GND            ->  GND
    SCL            ->  3(SCL2)
    SDA            ->  4(SDA2)
    INT            ->  A3

The USFSMAX board can also be used with either of the other two I2C ports and a different DRDY interrupt but the "INT_PIN" and "SENSOR_0_WIRE_INSTANCE" definitions in the "config.h" tab would need updating to reflect the changes.

## Building/Uploading the Test Sketch
