/**
 * @file    VCNL4040.cpp
 * @version 0.01 Alpha
 * @brief   Library for the VCNL4040
 * @author  Steven
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Vishay VCNL4040 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library.
 * This library is heavily modified from SparkFun's APDS9960 Library
 */

 #include <Arduino.h>
 #include <Wire.h>

 #include "VCNL4040.h"

/**
 * @brief Constructor - Instantiates VCNL4040 object
 */
VCNL4040::VCNL4040()
{

}

/**
 * @brief Destructor
 */
VCNL4040::~VCNL4040()
{

}

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
bool VCNL4040::init()
{
    uint8_t id;
    /* Initialize I2C */
    Wire.begin();
    /* Read ID register and check against known values for VCNL4040 */
    if( !wireRead8(VCNL4040_ID_L, id) ) {
        return false;
    }
    return true;
}

//Settings
bool VCNL4040::alsConf(uint8_t conf)
{
  if(!wireWrite8(VCNL4040_ALS_CONF, conf))
  {
    return false;
  }
  return true;
}

uint16_t VCNL4040::lux(void)
{
  uint16_t counts;
  if(!wireRead16(VCNL4040_ALS_DATA, counts))
  {
    return -1;
  }
  return counts/20;
}

bool VCNL4040::psConf(uint8_t conf1, uint8_t conf2, uint8_t conf3, uint8_t ms)
{
  uint16_t confReg1 = conf2;
  confReg1 = (confReg1 << 8) + conf1;

  uint16_t confReg2 = ms;
  confReg2 = (confReg2 << 8) + conf3;

  if(!wireWrite16(VCNL4040_PS_CONF_1_2, confReg1))
  {
    return false;
  }

  if(!wireWrite16(VCNL4040_PS_CONF_3_MS, confReg2))
  {
    return false;
  }
  return true;
}

uint16_t VCNL4040::psIdle()
{
  uint32_t tally=0;
  for (uint8_t i=0; i<15; i++){
    tally+=ps();
  }

  return (uint16_t)(tally/16);

}

uint16_t VCNL4040::psCalibrate()
{
  uint32_t tally = 0;
  for (uint8_t i = 0; i < 8; i++) {
		tally+=ps();
	}
  tally = tally/8;
  return (uint16_t)tally;
}

bool VCNL4040::psSetCanc(uint16_t level)
{
  if(!wireWrite16(VCNL4040_PS_CANC, level))
  {
    return false;
  }
  return true;
}

bool VCNL4040::psSetIntThres(uint16_t high, uint16_t low)
{
  if(!wireWrite16(VCNL4040_PS_THDH, high))
  {
    return false;
  }

  if(!wireWrite16(VCNL4040_PS_THDL, low))
  {
    return false;
  }

  return true;
}
uint16_t VCNL4040::ps(void)
{
  uint16_t ps;
  if(!wireRead16(VCNL4040_PS_DATA, ps))
  {
    return -1;
  }
  return ps;
}

uint16_t VCNL4040::intFlag(void)
{
  uint16_t intF;
  if(!wireRead16(VCNL4040_INT_FLAG, intF))
  {
    return -1;
  }

  return (uint16_t)intF;
}

uint8_t VCNL4040::id(void)
{
  uint8_t id;
  if(!wireRead8(VCNL4040_ID, id))
  {
    return -1;
  }
  return id;
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

bool VCNL4040::wireWrite8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(VCNL4040_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }
    return true;
}

bool VCNL4040::wireWrite16(uint8_t reg, uint16_t val)
{
  uint8_t LSB = (uint8_t)(0xFF & val);
  uint8_t MSB = (uint8_t)(val >> 8);
  Wire.beginTransmission(VCNL4040_I2C_ADDR);
  Wire.write(reg);
  Wire.write(LSB);
  Wire.write(MSB);
  if( Wire.endTransmission() != 0 ) {
      return false;
  }
  return true;
}

bool VCNL4040::wireReadStart(uint8_t val)
{
    Wire.beginTransmission(VCNL4040_I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission(false) != 0 ) {
        return false;
    }

    return true;
}

bool VCNL4040::wireRead8(uint8_t reg, uint8_t &val)
{

    /* Indicate which register we want to read from */
    if (!wireReadStart(reg)) {
        return false;
    }
    /* Read from register */
    Wire.requestFrom(VCNL4040_I2C_ADDR, 1);
    if (1<=Wire.available())
    {
      while (!Wire.available());
      val = Wire.read();
      Wire.endTransmission();
      return true;
    }
    Wire.endTransmission();
    return false;
}

bool VCNL4040::wireRead16(uint8_t reg, uint16_t &val)
{
    uint16_t LSB;
    uint16_t MSB;
    /* Indicate which register we want to read from */
    if (!wireReadStart(reg)) {
        return false;
    }

    /* Read block data */
    Wire.requestFrom(VCNL4040_I2C_ADDR, 2);
    if (2 <= Wire.available())
    {
      while (!Wire.available());
      LSB = Wire.read();
      MSB = Wire.read();
      MSB = MSB << 8;
      val = MSB | LSB;
      Wire.endTransmission();
      return true;
    }
    Wire.endTransmission();
    return false;
}
