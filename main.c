//==============================================================================
// Harvard University: adapted from SENSIRION AG Sample Code
//==============================================================================
// Project   :  Temperature and Humidity Data Recording
// File      :  sht85_bcm2835.c
// Author    :  NEC
// Date      :  Apr 2019
// Controller:  Raspberry Pi BCM2835
// Brief     :  Testing and implementing sensor access
//==============================================================================

#include "sht85_bcm2835.h"
#include <bcm2835.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


//------------------------------------------------------------------------------
int main(void)
{
  etError  error;          // error code
  uint32_t serialNumber;   // serial number
  float    temperature;    // temperature [°C]
  float    humidity;       // relative humidity [%RH]
  
  int timeout = 20;
  
  SHT85_Init();
  
  // wait 50ms after power on
  delay(50);    
  
  
  // demonstartion of SoftReset command
  error = SHT85_SoftReset();
  
  // demonstartion of ReadSerialNumber command
  error = SHT85_ReadSerialNumber(&serialNumber);
  printf("Serial number: %d \n", serialNumber);
  
  // demonstration of the single shot measurement
  // measurement with high repeatability
  error = SHT85_SingleMeasurment(&temperature, &humidity, SINGLE_MEAS_HIGH, 50);
  if (error == NO_ERROR){
  printf("Check temp: %f \n Check hum: %f \n", temperature, humidity);
}

 
  // --- demonstration of the periodic measurement mode ---
  while(timeout--) {
    // start periodic measurement, with high repeatability and 1 measurements
    // per second
    error = SHT85_StartPeriodicMeasurment(PERI_MEAS_HIGH_1_HZ);

    
    // loop while no error
    while(error == NO_ERROR) {
      // read measurment buffer
      error = SHT85_ReadMeasurementBuffer(&temperature, &humidity);
      
      if(error == NO_ERROR) {
        printf("Temp %f, Hum %f \n", temperature, humidity);
      } else if (error == ACK_ERROR) {
        error = NO_ERROR;
        // there were no new values in the buffer -> ignore this error
      } else {
        // exit loop on all other errors
        break;
      }
      
      // wait 100ms
    delay(100);
    }
    
    // --- error handling ---
    
    // ... and perfom a soft reset
    error = SHT85_SoftReset();
    
    // if the soft reset was not successful, perform an general call reset
    // (to implement)
    
    // wait 100ms
    delay(100);
  }
  
  
  SHT85_I2C_Exit();
}


