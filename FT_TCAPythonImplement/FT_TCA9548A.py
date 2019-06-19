import Adafruit_GPIO.FT232H as ft
import Adafruit_GPIO.GPIO as gpio # this might be redundant
import struct
import logging
import time
import binascii

from SHTSensor_I2C import SHTSensor_I2C as SHTSensor

# Set this up a a subclass of I2CDevice from FT232H

_DEFAULT_ADDRESS = 0x70


class TCA9548A():
    """Class which provides interface to TCA9548A I2C multiplexer."""

    def __init__(self, i2c, address=_DEFAULT_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.channels = [None]*8
        self.tca_device = ft.I2CDevice(i2c, address)

    def __len__(self):
        return 8

    def tca_init(self, chan):
        if (chan > 7):
            print("Warning:selected channel out of range")
            return

        logging.debug("Setting channel: " + str(chan))
        if self.channels[chan] is None:
            #  write a byte to TCA at default address to switch the channel
            print( hex(1 << chan) )
            self.tca_device.writeRaw8( 1 << int(chan) )
            self.channels[chan] = SHTSensor(self.i2c, 0x44)
            #TCA9548A_Channel(serial=serialDev )

        return self.channels[chan]
 
    def tca_select(self, chan)
        self.tca_device.writeRaw8 ( 1<< int(chan) )


