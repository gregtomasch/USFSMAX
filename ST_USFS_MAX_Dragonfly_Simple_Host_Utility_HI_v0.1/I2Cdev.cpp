#include "Arduino.h"
#include "I2Cdev.h"

I2Cdev::I2Cdev(TwoWire* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

/**
* @fn: readByte(uint8_t address, uint8_t subAddress)
*
* @brief: Read one byte from an I2C device
* 
* @params: I2C slave device address, Register subAddress
* @returns: unsigned short read
*/
uint8_t I2Cdev::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t temp;
  
  _i2c_bus->transfer(address, &subAddress, 1, &temp, 1);
  return temp;

}

/**
* @fn: readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
*
* @brief: Read multiple bytes from an I2C device
* 
* @params: I2C slave device address, Register subAddress, number of btes to be read, aray to store the read data
* @returns: void
*/
void I2Cdev::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
 _i2c_bus->transfer(address, &subAddress, 1, dest, count); 
}

/**
* @fn: writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
*
* @brief: Write one byte to an I2C device
* 
* @params: I2C slave device address, Register subAddress, data to be written
* @returns: void
*/
void I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  uint8_t temp[2];
  
  temp[0] = regAddr;
  temp[1] = data;
  _i2c_bus->transfer(devAddr, &temp[0], 2, NULL, 0); 
}

/**
* @fn: writeBytes(uint8_t device_address, uint8_t regAddr, uint8_t count, uint8_t *source)
*
* @brief: Write one byte to an I2C device
* 
* @params: I2C slave device address, Register subAddress, byte count to be written data array to be written
* @returns: void
*/
void I2Cdev::writeBytes(uint8_t device_address, uint8_t regAddr, uint8_t count, uint8_t *source)
{
  uint8_t temp[1 + count];
  
  temp[0] = regAddr;
  for (uint8_t ii = 0; ii < count; ii++)
  { 
    temp[ii + 1] = source[ii];
  }
  _i2c_bus->transfer(device_address, temp, (count + 1), NULL, 0);
}

/**
* @fn:I2Cscan()
* @brief: Scan the I2C bus for active I2C slave devices
* 
* @params: void
* @returns: void
*/
void I2Cdev::I2Cscan() 
{
  // Scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of the Wire.endTransmisstion to see if a device did acknowledge to the address.
    _i2c_bus->beginTransmission(address);
    error = _i2c_bus->endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("!");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("I2C scan complete\n");
}
