//==============================================================================
// Harvard University: adapted from SENSIRION AG Sample Code
//==============================================================================
// Project   :  Temperature and Humidity Data Recording
// File      :  sht85_bcm2835.c
// Author    :  NEC
// Date      :  Apr 2019
// Controller:  Raspberry Pi BCM2835
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//              to do: implement timer changes and error handling, add LED indicators
//==============================================================================

#include "sht85_bcm2835.h"
#include <bcm2835.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

// #include "system.h"

// BCM constants
#define MODE_READ 0
#define MODE_WRITE 1
#define MAX_LEN 32

// SHT constants
#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define I2C_ADDR        0x44

// function def
static uint8_t CalcCrc(uint8_t data[], uint8_t nbrOfBytes);
static etError CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static float CalcTemperature(uint16_t rawValue);
static float CalcHumidity(uint16_t rawValue);


/* Although the BCM microcontroller is capable of combining read/write 
   commands, the SHT85 does not seem to suppor this mode (?)    */

//------------------------------------------------------------------------------
void SHT85_Init(void)
{
  if (!bcm2835_init())
  {
    printf("bcm2835_init failed\n");
  }
  
  bcm2835_i2c_setSlaveAddress(I2C_ADDR);
  
  int bcError = bcm2835_i2c_begin();
  
  if (!bcError) {
	  printf("bcm2835 I2C protocol failure \n");
  }
  
}

// ---------------

void SHT85_Exit(void)
{
	bcm2835_i2c_end();
}


//------------------------------------------------------------------------------
etError SHT85_ReadSerialNumber(uint32_t *serialNumber)
{
  etError error;
  char cmd[2];  
  
  cmd[0] = CMD_READ_SERIALNBR >> 8 & 0xFF;
  cmd[1] = CMD_READ_SERIALNBR & 0xFF;
  
  char serialData[5];
  
	error = bcm2835_i2c_write(cmd, 2); 
	delay(5);
	if (error == NO_ERROR) {
		error = bcm2835_i2c_read(serialData,5);  
    }
 
	// if no error, calc serial number as 32-bit integer
	if (error == NO_ERROR){
		*serialNumber = (serialData[0] << 24) ;
		*serialNumber |= serialData[1] << 16;
		*serialNumber |= serialData[2] << 8;
		*serialNumber |= serialData[3];  
			
		
		uint8_t checkSum = serialData[4];
		error = CheckCrc(serialData, 4, checkSum);
	}
  return error;

    
}

//------------------------------------------------------------------------------
etError SHT85_ReadStatus(uint16_t* status)
{

  etError error;
  char cmd[2];
  cmd[0] = CMD_READ_STATUS >> 8 & 0xFF;
  cmd[1] = CMD_READ_STATUS & 0xFF;
  
  char serialData[2];

  error = bcm2835_i2c_write(cmd, 2); 
  delay(5);
    
  if (error == NO_ERROR) {
      error = bcm2835_i2c_read(serialData,3);  //
  }
  
  *status =   (serialData[0] << 8) | serialData[1] ;
  
  return error;
  }

//------------------------------------------------------------------------------
etError SHT85_ClearAllAlertFlags(void)
{
  etError error; // error code
  char cmd[2];
  
  cmd[0] = CMD_CLEAR_STATUS >> 8 & 0xFF;
  cmd[1] = CMD_CLEAR_STATUS & 0xFF;
  
  error = bcm2835_i2c_write(cmd, 2);   
     
    delay(2);
    return error;
    

}

//------------------------------------------------------------------------------
etError SHT85_SingleMeasurment(float* temperature, float* humidity,
                               etSingleMeasureModes measureMode,
                               uint8_t timeout)
{
  etError  error;           // error code
  
  char modeSet[2];
  char readData[6];
  char tempData[2];
  char humData[2];
  
  uint8_t crcT, crcH;
  uint16_t rawTemp, rawHum;
  
  
  modeSet[0] = (etCommands)measureMode >> 8 & 0xFF;
  modeSet[1] = (etCommands)measureMode & 0xFF;
  
  error = bcm2835_i2c_write(modeSet, 2);  // Set single measurement mode
  delay(10); // delay between commands: minimum of 2ms    

  if (error == NO_ERROR) 
  {
    error = bcm2835_i2c_read(readData,6); // read 6 bytes of data (2+CRC x2)  
      
    if (error == NO_ERROR) 
    {
		rawTemp = (readData[0] << 8 | readData[1]);  
		crcT = readData[2];
	    rawHum = (readData[3] << 8 | readData[4]);
        crcH = readData[5];

        *temperature = CalcTemperature(rawTemp);
        *humidity = CalcHumidity(rawHum);
        
        // check for errors
        memcpy(tempData, readData, 2);
        memcpy(humData, readData + 3, 2);
    
        error = CheckCrc(tempData,2,crcT);
        if (error == NO_ERROR) { 
			error = CheckCrc(humData,2,crcH);}	   
	   } 
    }
    return error;

}

//------------------------------------------------------------------------------
etError SHT85_StartPeriodicMeasurment(etPeriodicMeasureModes measureMode)
{
  etError error; // error code
  char modeSet[2];
  modeSet[0] = (etCommands)measureMode >> 8 & 0xFF;
  modeSet[1] = (etCommands)measureMode & 0xFF;
  
    error = bcm2835_i2c_write(modeSet, 2);
    delay(2);
    return error;
}



//------------------------------------------------------------------------------
etError SHT85_StopPeriodicMeasurment(void)
{
  etError error; // error code
  
  char cmd[2];
  cmd[0] = CMD_BREAK >> 8 & 0xFF;
  cmd[1] = CMD_BREAK & 0xFF;
  
   error = bcm2835_i2c_write(cmd, 2);
      delay(2);
  
    return error;
}
  


//------------------------------------------------------------------------------
etError SHT85_ReadMeasurementBuffer(float* temperature, float* humidity)
{
  etError  error;        // error code
  uint16_t rawTemp, rawHum;
  char cmd[2];  
  char readData[6];
  char tempData[2], humData[2];
  uint8_t crcT, crcH;
  
  cmd[0] = CMD_FETCH_DATA >> 8 & 0xFF;
  cmd[1] = CMD_FETCH_DATA & 0xFF;
  
  error = bcm2835_i2c_write(cmd, 2);

      if (error == NO_ERROR) {
        delay(2);  // set delay to minimum   
        error = bcm2835_i2c_read(readData,6); // read 6 bytes of data (2+CRC x2)
        
        if (error == NO_ERROR) {
		// this could be spun off to a subfunction with temp, hum, readData
		// use for single or continuous measurement 
 
			rawTemp = (readData[0] << 8 | readData[1]);  
			crcT = readData[2];
			rawHum = (readData[3] << 8 | readData[4]);
			crcH = readData[5];
	
			*temperature = CalcTemperature(rawTemp);
			*humidity = CalcHumidity(rawHum);
 
			
    	memcpy(tempData, readData, 2);
		memcpy(humData, readData + 3, 2);
			
		error = CheckCrc(tempData,2,crcT);
		if (error == NO_ERROR) {  error = CheckCrc(humData,2,crcH);}
			
		}
  	  }



}

//------------------------------------------------------------------------------
etError SHT85_EnableHeater(void)
{
  etError error; // error code
  
  char cmd[2];
  cmd[0] = CMD_HEATER_ENABLE >> 8 & 0xFF;
  cmd[1] = CMD_HEATER_ENABLE & 0xFF;

  error = bcm2835_i2c_write(cmd, 2);
  
    delay(2);

    return error;
}

//------------------------------------------------------------------------------
etError SHT85_DisableHeater(void)
{
  etError error; // error code  

  char cmd[2];
  cmd[0] = CMD_HEATER_DISABLE >> 8 & 0xFF;
  cmd[1] = CMD_HEATER_DISABLE & 0xFF;

    error = bcm2835_i2c_write(cmd, 2);
    
    return error;
}

//------------------------------------------------------------------------------
etError SHT85_SoftReset(void)
{
  etError error; // error code  
  char cmd[2];
  cmd[0] = CMD_SOFT_RESET >> 8 & 0xFF;
  cmd[1] = CMD_SOFT_RESET & 0xFF;

  error = bcm2835_i2c_write(cmd, 2);
  
  delay(50);
  return error;
}


//------------------------------------------------------------------------------
static uint8_t CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }
  
  return crc;
}

//------------------------------------------------------------------------------
static etError CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  // calculates 8-Bit checksum
  uint8_t crc = CalcCrc(data, nbrOfBytes);
  
  
  // verify checksum
  return (crc != checksum) ? CHECKSUM_ERROR : NO_ERROR;
}

//------------------------------------------------------------------------------
static float CalcTemperature(uint16_t rawValue)
{
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

//------------------------------------------------------------------------------
static float CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}
