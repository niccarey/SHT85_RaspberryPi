# Class and methods for SHT85 Sensiron humidity and temperature sensor

""" To do:    find out i2c error codes
"""


# import libraries
import atexit
import logging
import math
import os
import sys
import time
import binascii

# external note: MUX address is 0x70
# import ftdi1 as ftdi

# breakout board
import Adafruit_GPIO.FT232H as ft
import Adafruit_GPIO.GPIO as gpio # this might be redundant

import SHTConstants

SHT_ADDR = 0x44
CRC_POLY = 0x131

# Unclear why this needs to be in two places
# definitions
DEFAULT_DELAY = 0.002 # 2 milliseconds ??

# error codes
# NO_ERROR = i2c ucontroller no error return
# ACK_ERROR = ditto NACK
# CHECKSUM_ERROR = 0x08 *unless taken
# TIMEOUT_ERROR = i2c ucontroller timeout error
# DATA_ERROR = ditto reason error

# User-facing constants
CMD_READ_SERIALNBR  = 0x3780
CMD_READ_STATUS     = 0xF32D
CMD_CLEAR_STATUS    = 0x3041
CMD_HEATER_ENABLE   = 0x306D
CMD_HEATER_DISBLE   = 0x3066
CMD_SOFT_RESET      = 0x30A2
CMD_MEAS_SINGLE_H   = 0x2400
CMD_MEAS_SINGLE_M   = 0x240B
CMD_MEAS_SINGLE_L   = 0x2416
CMD_MEAS_PERI_05_H  = 0x2032
CMD_MEAS_PERI_05_M  = 0x2024
CMD_MEAS_PERI_05_L  = 0x202F
CMD_MEAS_PERI_1_H   = 0x2130
CMD_MEAS_PERI_1_M   = 0x2126
CMD_MEAS_PERI_1_L   = 0x212D
CMD_MEAS_PERI_2_H   = 0x2236
CMD_MEAS_PERI_2_M   = 0x2220
CMD_MEAS_PERI_2_L   = 0x222B
CMD_MEAS_PERI_4_H   = 0x2334
CMD_MEAS_PERI_4_M   = 0x2322
CMD_MEAS_PERI_4_L   = 0x2329
CMD_MEAS_PERI_10_H  = 0x2737
CMD_MEAS_PERI_10_M  = 0x2721
CMD_MEAS_PERI_10_L  = 0x272A
CMD_FETCH_DATA      = 0xE000
CMD_BREAK           = 0x3093

SINGLE_MEAS_LOW     = CMD_MEAS_SINGLE_L    # low repeatability
SINGLE_MEAS_MEDIUM  = CMD_MEAS_SINGLE_M    # medium repeatibility
SINGLE_MEAS_HIGH    = CMD_MEAS_SINGLE_H    # high repeatability

PERI_MEAS_LOW_05_HZ   = CMD_MEAS_PERI_05_L
PERI_MEAS_MED_05_HZ   = CMD_MEAS_PERI_05_M
PERI_MEAS_HIGH_05_HZ  = CMD_MEAS_PERI_05_H
PERI_MEAS_LOW_1_HZ    = CMD_MEAS_PERI_1_L
PERI_MEAS_MED_1_HZ    = CMD_MEAS_PERI_1_M
PERI_MEAS_HIGH_1_HZ   = CMD_MEAS_PERI_1_H
PERI_MEAS_LOW_2_HZ    = CMD_MEAS_PERI_2_L
PERI_MEAS_MED_2_HZ    = CMD_MEAS_PERI_2_M
PERI_MEAS_HIGH_2_HZ   = CMD_MEAS_PERI_2_H
PERI_MEAS_LOW_4_HZ    = CMD_MEAS_PERI_4_L
PERI_MEAS_MED_4_HZ    = CMD_MEAS_PERI_4_M
PERI_MEAS_HIGH_4_HZ   = CMD_MEAS_PERI_4_H
PERI_MEAS_LOW_10_HZ   = CMD_MEAS_PERI_10_L
PERI_MEAS_MED_10_HZ   = CMD_MEAS_PERI_10_M
PERI_MEAS_HIGH_10_HZ  = CMD_MEAS_PERI_10_H


# mux stuff
# need to somehow wrap this stuff inside the MUX communication

# FT232H generic read/write commands expect register addressing, with no obvious way
# to turn this off, so we use low-level commands for the SHT interface
# SHT does not require active idle state

class SHTSensor_I2C:

    # Command byte structure, not strictly necessary
    _BUFFER = bytearray(2) # check this is valid for python 2.7

    def __init__(self, i2cBus, address=SHT_ADDR):
        self.address = address

        # Initialise the I2C instance for the sensor - 
        # default is first FT232H instance but when muxing will be different
        self._device = ft.I2CDevice(i2cBus, address)
        time.sleep(0.002)
        if not (self._device.ping()):
            print("Problem accessing device at " + hex(address))


    def ReadSerialNumber(self):
        # CURRENTLY BUGGY: might expect to return 2x(byte+crc)
        # See Sensirion sample code for implementation

        # write command to register:
        self.writeCom(CMD_READ_SERIALNBR, 2)
        time.sleep(0.002)

        # read five bytes from i2C register
        serialData = self.readBytes(5)

        serialNumber  = (serialData[0] << 24)
        serialNumber |= (serialData[1] << 16)
        serialNumber |= (serialData[2] << 8)
        serialNumber |= serialData[3];

        crcByte = serialData[4]

        # if no error is returned, convert bytes into serial number + CRC
        # check CRC
        logging.debug('CRC check: ' + hex(crcByte))
        errorBool = self.checkCrc(serialData, 4, crcByte)
        logging.debug('checksum ok? ' + str(errorBool))

        return serialNumber

    def softReset(self):
        # send soft reset command
        self.writeCom(CMD_SOFT_RESET,2)
        time.sleep(0.002)

    def clearFlags(self):
        # write command to i2c as 2-byte series
        self.writeCom(CMD_CLEAR_STATUS,2)
        time.sleep(0.002)
        # delay, check error


    def readStatus(self):
        # write command to i2c as 2-byte serie
        self.writeCom(CMD_READ_STATUS,2)
        time.sleep(0.002)
        logging.debug('Reading status result')
        serialData = self.readBytes(3)

        # convert to status signal and CRC 
        status =  (serialData[0] << 8) | (serialData[1] )
        # check CRC
        crcByte = serialData[2]

        logging.debug('CRC check: ' + str(crcByte))
        errorBool = self.checkCrc(serialData, 2, crcByte)
        logging.debug('checksum ok? ' + str(errorBool))

        return status


    def SingleMeasurement(self, measureMode, timeout):
        self.writeCom(measureMode, 2)
        time.sleep(0.01)

        readData = self.readBytes(6)

        time.sleep(0.1)
        rawTemp = (readData[0] << 8 | readData[1])
        crcT = readData[2];
        rawHum = (readData[3] << 8 | readData[4]);
        crcH = readData[5];

        temperature = self.CalcTemperature(rawTemp);
        humidity = self.CalcHumidity(rawHum);
        tempcheck = bytearray(2)
        humcheck = bytearray(2)

        tempcheck[0] = readData[0]
        tempcheck[1] = readData[1]
        humcheck[0] = readData[3]
        humcheck[1] = readData[4]

        logging.debug('Temp CRC check: ' + str(crcT))
        errorBool = self.checkCrc(tempcheck, 2, crcT)
        logging.debug('checksum ok? ' + str(errorBool))
        logging.debug('Humidity CRC check: ' + str(crcH))
        errorBool = self.checkCrc(humcheck, 2, crcH)
        logging.debug('checksum ok? ' + str(errorBool))

        return temperature, humidity


    def CalcTemperature(self, rawTemp):
        temp = 175.0*rawTemp/65535.0 - 45.0
        return temp

    def CalcHumidity(self, rawHumid):
        humid = 100.0*rawHumid/65535.0

        return humid


    def readBytes(self, nbrBytes):
        # why can't I use with??

        self._device._transaction_start()
        self._device._i2c_start()
        # send address with read bit set
        self._device._i2c_write_bytes([self._device._address_byte(True)])
        self._device._i2c_read_bytes(nbrBytes)
        self._device._i2c_stop()
        response = self._device._transaction_end()

        logging.debug('Read response: ' + binascii.hexlify(response))
        # this is sending back garbage because the SHT doesn't bounce ACK bits
        # self._device._verify_acks(response[:-1])

        data_return = response
        del data_return[0]
        # knock one byte off response to deal with address return
        return data_return


    def writeCom(self, command, nbrBytes):
        # convert command into a series of bytes
        cmd = bytearray(nbrBytes+1)
        # we are using low-level write commands so bundle the address with the command
        cmd[0] = self._device._address_byte(False)

        #self._device.writeRaw8(self._device._address_byte(True))
        for bIn in range (1, nbrBytes+1):
            cmd[bIn] = command >> 8*(nbrBytes-bIn) & 0xFF

        # write to i2C, delay at least 2ms
        self._device._transaction_start()
        self._device._i2c_start()
        self._device._i2c_write_bytes(cmd)
        self._device._i2c_stop()

        response = self._device._transaction_end()
        time.sleep(0.002)

        logging.debug('Write Response: ' + binascii.hexlify(response))

        self._device._verify_acks(response)

    def checkCrc(self, data, nbrBytes, checksum):
        # send data, nbrBytes to calcCRC - something funky here
        crc = self.calcCrc(data, nbrBytes)
        logging.debug('checksum: ' + hex(checksum))
        logging.debug('crc response: ' + hex(crc))

        # verify checksum
        if crc != checksum:
            return False
        else: return True


    def calcCrc(self, data, nbrBytes):
        # bit shifting using polynomial
        crc = 0xFF

        for byteCt in range(0, nbrBytes):
            print(hex(crc))

            crc ^= (data[byteCt])
            for bit in range(8,0,-1): 
                if(crc & 0x80):
                    crc = (crc << 1) ^ CRC_POLY
                else: crc = (crc << 1) 

        print(hex(crc))
        return crc;





"""  STILL TO CODE:
def StartPeriodicMeasurement(self, measureMode):
        # send measureMode as byte chunks


    def StopPeriodicMeasurement(self):
        # send measureMode as byte chunks

    def ReadMeasurementBuffer(self):
        # send fetch data command
        # read 6 bytes
        # convert to rawtemp, tCRC, rawhumidity, hCRC
        # check CRC values
        # convert to real temp, real humidity


    def enableHeater(self):
        # send enable heater command


    def disableHeater(self):
        # send disable heater command

"""




