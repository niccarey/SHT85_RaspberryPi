# SHT85_RaspberryPi
SHT-85 library and sample code for Raspberry Pi running BCM2835. 

This sample project shows the basic usage of Sensirion's SHT85 temperature and humidity sensor from a Raspberry Pi

Contains:
- 'sht_bcm2835.c' C library for implementing SHT read and status functions on the raspberry pi
- 'sht_bcm2835.h' library header file
- 'main.c' basic functionality test

To run: make sure you have gcc installed. Run 

gcc sht85_bcm2835.c main.c -I/home/pi/PathToSHTSensorLib -l bcm2835 -o functionTest

to compile. Note that any program calling the bcm2835 library must be run as root.

## Compatibility and requirements
The code herein is compatible with Sensirion's SHT85 digital temperature and humidity sensor. It requires installation of the Broadcom BCM2835 C library, https://www.airspayce.com/mikem/bcm2835/index.html.

## Sensirion SHT85 Sensor
BCM2835 Board

Sensirion's developer page developer.sensirion.com provides more developer resources for different platforms and products.
