/*!
 *  @file Adafruit_INA2xx.cpp
 *
 *  @section ina2xx_intro Introduction
 *
 * 	I2C Driver base class for the INA2xx I2C Current and Power sensors
 *
 * 	This is a library for the Adafruit INA228 breakout:
 * 	http://www.adafruit.com/products/5832
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section ina2xx_dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section ina2xx_author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section ina2xx_license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section ina2xx_history HISTORY
 *
 *     v1.0 - First release
 */

#include "ina228/Adafruit_INA2xx.h"

/*!
 *    @brief  Instantiates a new INA2xx class
 */
namespace ina228 {
Adafruit_INA2xx::Adafruit_INA2xx(void) {}

/*!
 *    @brief  Sets up the HW
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  theWire
 *            The Wire object to be used for I2C connections.
 *    @param  skipReset
 *            When set to true, will omit resetting all registers to
 *            their default values. Default: false.
 *    @return True if initialization was successful (including verifying
 *            the manufacturer ID is Texas Instruments: 0x5449), otherwise
 * false.
 */
Adafruit_INA2xx::Adafruit_INA2xx(std::string i2c_bus, uint8_t i2c_addr) : i2c_bus_(i2c_bus.c_str()), i2c_addr_(i2c_addr) {}
Adafruit_INA2xx::~Adafruit_INA2xx()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}
bool Adafruit_INA2xx::begin(bool skipReset) {
  // Open i2c port
  if((fd_ = open(i2c_bus_, O_RDWR)) < 0) {
    return false;
  }
  // Flow control
  if(ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
    return false;
  }
  return true;

  /*Adafruit_I2CRegister* device_register =
      new Adafruit_I2CRegister(i2c_dev, INA2XX_REG_DVC_UID, 2, MSBFIRST);
  Adafruit_I2CRegister* mfg_register =
      new Adafruit_I2CRegister(i2c_dev, INA2XX_REG_MFG_UID, 2, MSBFIRST);
  Adafruit_I2CRegisterBits* device_id =
      new Adafruit_I2CRegisterBits(device_register, 12, 4);*/

  // Check manufacturer ID (should be 0x5449 for Texas Instruments)
  uint16_t mfg_id = read_register_16bit(INA2XX_REG_MFG_UID);
  if (mfg_id != 0x5449) {
    return false;
  }

  // Store device ID for validation in derived classes
  //_device_id = device_id->read();

  /*Config = new Adafruit_I2CRegister(i2c_dev, INA2XX_REG_CONFIG, 2, MSBFIRST);
  //ADC_Config =
  //    new Adafruit_I2CRegister(i2c_dev, INA2XX_REG_ADCCFG, 2, MSBFIRST);
  Diag_Alert =
      new Adafruit_I2CRegister(i2c_dev, INA2XX_REG_DIAGALRT, 2, MSBFIRST);

  if (!skipReset) {
    reset();
    delay(2); // delay 2ms to give time for first measurement to finish
  }
  return true;
  */
 if(!skipReset) {
  reset();
  usleep(2000);
 }
}
/**************************************************************************/
/*!
    @brief Resets the hardware. All registers are set to default values,
    the same as a power-on reset.
*/
/**************************************************************************/
void Adafruit_INA2xx::reset(void) {
  uint16_t config_val = read_register_16bit(INA2XX_REG_CONFIG);
  config_val |= (1<<15); // Set RST Bit
  write_register_16bit(INA2XX_REG_CONFIG, config_val);
  setMode(INA2XX_MODE_CONTINUOUS);
  /*
  Adafruit_I2CRegisterBits reset = Adafruit_I2CRegisterBits(Config, 1, 15);
  reset.write(1);
  Adafruit_I2CRegisterBits alert_conv =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 14);
  alert_conv.write(1);
  setMode(INA2XX_MODE_CONTINUOUS);*/
}

/**************************************************************************/
/*!
    @brief Updates the shunt calibration value to the register.
    This is implemented in the derived classes due to different calculations
*/
/**************************************************************************/
void Adafruit_INA2xx::_updateShuntCalRegister() {
  // Implemented in derived classes
}

/**************************************************************************/
/*!
    @brief Sets the shunt calibration by resistor.
    @note This base implementation uses INA228 settings.
          Derived classes override this method for their specific settings.
    @param shunt_res Resistance of the shunt in ohms (floating point)
    @param max_current Maximum expected current in A (floating point)
*/
/**************************************************************************/
void Adafruit_INA2xx::setShunt(float shunt_res, float max_current) {
  _shunt_res = shunt_res;
  // Default to INA228 behavior (2^19 divisor)
  _current_lsb = max_current / (float)(1UL << 19);
  _updateShuntCalRegister();
}

/**************************************************************************/
/*!
    @brief Sets the shunt full scale ADC range across IN+ and IN-.
    @param adc_range
          Shunt full scale ADC range (0: +/-163.84 mV or 1: +/-40.96 mV)
*/
/**************************************************************************/
void Adafruit_INA2xx::setADCRange(uint8_t adc_range) {
  uint16_t config_val = read_register_16bit(INA2XX_REG_CONFIG);

  if (adc_range == 0) {
    // Clear bit 4 for +/-163.84 mV range
    config_val &= ~(1 << 4);
  } else {
    // Set bit 4 for +/-40.96 mV range
    config_val |= (1 << 4);
  }

  write_register_16bit(INA2XX_REG_CONFIG, config_val);
  /*
  Adafruit_I2CRegisterBits adc_range_bit =
      Adafruit_I2CRegisterBits(Config, 1, 4);
  adc_range_bit.write(adc_range);
  _updateShuntCalRegister();*/
}

/**************************************************************************/
/*!
    @brief Reads the shunt full scale ADC range across IN+ and IN-.
    @return Shunt full scale ADC range (0: +/-163.84 mV or 1: +/-40.96 mV)
*/
/**************************************************************************/
uint8_t Adafruit_INA2xx::getADCRange() {
  int16_t config_val = read_register_16bit(INA2XX_REG_CONFIG);
  return (config_val >> 4) & 1;
}

/**************************************************************************/
/*!
    @brief Reads the die temperature (using INA228 scale factor by default)
    @note This base implementation uses the INA228 conversion factor of 7.8125
   m°C/LSB. Derived classes override this method as needed (e.g., INA237).
    @return The current die temp in deg C
*/
/**************************************************************************/
float Adafruit_INA2xx::readDieTemp(void) {
  /*Adafruit_I2CRegister temp =
      Adafruit_I2CRegister(i2c_dev, INA2XX_REG_DIETEMP, 2, MSBFIRST);
  int16_t t = temp.read();*/
  int16_t t = read_register_16bit(INA2XX_REG_DIETEMP);
  // INA228 uses 16 bits for temperature with 7.8125 m°C/LSB
  return (float)t * 7.8125 / 1000.0;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Current register.
    @return The current current measurement in mA
*/
/**************************************************************************/
float Adafruit_INA2xx::readCurrent(void) {
  int32_t i = read_register_24bit_signed(INA2XX_REG_CURRENT);
  /*Adafruit_I2CRegister current =
      Adafruit_I2CRegister(i2c_dev, INA2XX_REG_CURRENT, 3, MSBFIRST);
  int32_t i = current.read();*/
  /*if (i & 0x800000) signed logic
    i |= 0xFF000000;*/
  return (float)i / 16.0 * _current_lsb * 1000.0;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Current register.
    @return The current current measurement in mA
*/
/**************************************************************************/
float Adafruit_INA2xx::getCurrent_mA(void) {
  return readCurrent();
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Bus Voltage register.
    @note This base implementation uses the INA228 conversion factor.
          Derived classes override this method for their specific conversion
   factors.
    @return The current bus voltage measurement in V
*/
/**************************************************************************/
float Adafruit_INA2xx::readBusVoltage(void) {
  uint32_t bus_voltage = read_register_24bit_unsigned(INA2XX_REG_VBUS);
  /*Adafruit_I2CRegister bus_voltage =
      Adafruit_I2CRegister(i2c_dev, INA2XX_REG_VBUS, 3, MSBFIRST);*/
  // INA228 uses 195.3125 µV/LSB (microvolts) for bus voltage,
  // so we need to divide by 1e6 to get Volts
  return (float)((uint32_t)bus_voltage >> 4) * 195.3125 / 1e6;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Bus Voltage register.
    @return The current bus voltage measurement in V
*/
/**************************************************************************/
float Adafruit_INA2xx::getBusVoltage_V(void) {
  return readBusVoltage();
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Shunt Voltage register.
    @return The current shunt voltage measurement in mV
*/
/**************************************************************************/
float Adafruit_INA2xx::readShuntVoltage(void) {
  float scale = 312.5;
  if (getADCRange()) {
    scale = 78.125;
  }
  uint32_t v = read_register_24bit_signed(INA2XX_REG_VBUS);
  /*Adafruit_I2CRegister shunt_voltage =
      Adafruit_I2CRegister(i2c_dev, INA2XX_REG_VSHUNT, 3, MSBFIRST);
  int32_t v = shunt_voltage.read();*/
  /* signed logic
  if (v & 0x800000)
    v |= 0xFF000000;*/
  return (float)v / 16.0 * scale / 1000000.0;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Shunt Voltage register.
    @return The current shunt voltage measurement in mV
*/
/**************************************************************************/
float Adafruit_INA2xx::getShuntVoltage_mV(void) {
  return readShuntVoltage();
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Power register.
    @return The current Power calculation in mW
*/
/**************************************************************************/
float Adafruit_INA2xx::readPower(void) {
  /*Adafruit_I2CRegister power =
      Adafruit_I2CRegister(i2c_dev, INA2XX_REG_POWER, 3, MSBFIRST);*/
  uint32_t power = read_register_24bit_unsigned(INA2XX_REG_POWER);
  return (float)power * 3.2 * _current_lsb * 1000;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Power register.
    @return The current Power calculation in mW
*/
/**************************************************************************/
float Adafruit_INA2xx::getPower_mW(void) {
  return readPower();
}

/**************************************************************************/
/*!
    @brief Returns the current measurement mode
    @return The current mode
*/
/**************************************************************************/
INA2XX_MeasurementMode Adafruit_INA2xx::getMode(void) {
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  return (INA2XX_MeasurementMode)((adc_config >> 12) & 0xF);
}
/**************************************************************************/
/*!
    @brief Sets a new measurement mode
    @param new_mode
          The new mode to be set
*/
/**************************************************************************/
void Adafruit_INA2xx::setMode(INA2XX_MeasurementMode new_mode) {
  /*
  Adafruit_I2CRegisterBits mode = Adafruit_I2CRegisterBits(ADC_Config, 4, 12);
  mode.write(new_mode);
  */
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  adc_config &= ~(0xF << 12);       // Clear the mode bits (15:12)
  adc_config |= (new_mode << 12);   // Set the new mode
  write_register_16bit(INA2XX_REG_ADCCFG, adc_config);
}
/**************************************************************************/
/*!
    @brief Reads the current number of averaging samples
    @return The current number of averaging samples
*/
/**************************************************************************/
INA2XX_AveragingCount Adafruit_INA2xx::getAveragingCount(void) {
  /*
  Adafruit_I2CRegisterBits averaging_count =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 0);
  return (INA2XX_AveragingCount)averaging_count.read();
  */
 uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  return (INA2XX_AveragingCount)(adc_config & 0x7); // Bits 2:0
}
/**************************************************************************/
/*!
    @brief Sets the number of averaging samples
    @param count
          The number of samples to be averaged
*/
/**************************************************************************/
void Adafruit_INA2xx::setAveragingCount(INA2XX_AveragingCount count) {
  /*
    Adafruit_I2CRegisterBits averaging_count =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 0);
  averaging_count.write(count);
  */
 uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  adc_config &= ~0x7;               // Clear averaging bits (2:0)
  adc_config |= count;              // Set new averaging count
  write_register_16bit(INA2XX_REG_ADCCFG, adc_config);
}
/**************************************************************************/
/*!
    @brief Reads the current current conversion time
    @return The current current conversion time
*/
/**************************************************************************/
INA2XX_ConversionTime Adafruit_INA2xx::getCurrentConversionTime(void) {
  /*
  Adafruit_I2CRegisterBits current_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 6);
  return (INA2XX_ConversionTime)current_conversion_time.read();
  */
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  return (INA2XX_ConversionTime)((adc_config >> 6) & 0x7); // Bits 8:6
}
/**************************************************************************/
/*!
    @brief Sets the current conversion time
    @param time
          The new current conversion time
*/
/**************************************************************************/
void Adafruit_INA2xx::setCurrentConversionTime(INA2XX_ConversionTime time) {
  /*
    Adafruit_I2CRegisterBits current_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 6);
  current_conversion_time.write(time);
  */
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  adc_config &= ~(0x7 << 6);        // Clear the bits (8:6)
  adc_config |= (time << 6);        // Set the new time
  write_register_16bit(INA2XX_REG_ADCCFG, adc_config);
}
/**************************************************************************/
/*!
    @brief Reads the current bus voltage conversion time
    @return The current bus voltage conversion time
*/
/**************************************************************************/
INA2XX_ConversionTime Adafruit_INA2xx::getVoltageConversionTime(void) {
  /*Adafruit_I2CRegisterBits voltage_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 9);
  return (INA2XX_ConversionTime)voltage_conversion_time.read();*/
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  return (INA2XX_ConversionTime)((adc_config >> 9) & 0x7); // Bits 11:9
}
/**************************************************************************/
/*!
    @brief Sets the bus voltage conversion time
    @param time
          The new bus voltage conversion time
*/
/**************************************************************************/
void Adafruit_INA2xx::setVoltageConversionTime(INA2XX_ConversionTime time) {
  /*Adafruit_I2CRegisterBits voltage_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 9);
  voltage_conversion_time.write(time);*/
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  adc_config &= ~(0x7 << 9);        // Clear the bits (11:9)
  adc_config |= (time << 9);        // Set the new time
  write_register_16bit(INA2XX_REG_ADCCFG, adc_config);
  
}
/**************************************************************************/
/*!
    @brief Reads the temperature conversion time
    @return The temperature conversion time
*/
/**************************************************************************/
INA2XX_ConversionTime Adafruit_INA2xx::getTemperatureConversionTime(void) {
  /*Adafruit_I2CRegisterBits temperature_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 3);
  return (INA2XX_ConversionTime)temperature_conversion_time.read();*/
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  return (INA2XX_ConversionTime)((adc_config >> 3) & 0x7); // Bits 5:3
}
/**************************************************************************/
/*!
    @brief Sets the temperature conversion time
    @param time
          The new temperature conversion time
*/
/**************************************************************************/
void Adafruit_INA2xx::setTemperatureConversionTime(INA2XX_ConversionTime time) {
  /*dafruit_I2CRegisterBits temperature_conversion_time =
      Adafruit_I2CRegisterBits(ADC_Config, 3, 3);
  temperature_conversion_time.write(time);*/
  uint16_t adc_config = read_register_16bit(INA2XX_REG_ADCCFG);
  adc_config &= ~(0x7 << 3);        // Clear the bits (5:3)
  adc_config |= (time << 3);        // Set the new time
  write_register_16bit(INA2XX_REG_ADCCFG, adc_config);
}

/**************************************************************************/
/*!
    @brief Checks if the most recent one shot measurement has completed
    @return true if the conversion has completed
*/
/**************************************************************************/
bool Adafruit_INA2xx::conversionReady(void) {
  /*Adafruit_I2CRegisterBits conversion_ready =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 1);
  return conversion_ready.read();*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  return (diag_alrt >> 1) & 1; // Check CNVR bit (1)
}

/**************************************************************************/
/*!
    @brief Reads the current alert polarity setting
    @return The current bus alert polarity setting
*/
/**************************************************************************/
INA2XX_AlertPolarity Adafruit_INA2xx::getAlertPolarity(void) {
  /*Adafruit_I2CRegisterBits alert_polarity =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 12);
  return (INA2XX_AlertPolarity)alert_polarity.read();*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  return (INA2XX_AlertPolarity)((diag_alrt >> 12) & 1); // Check APOL bit (12)
}
/**************************************************************************/
/*!
    @brief Sets Alert Polarity Bit
    @param polarity
          The polarity of the alert pin
*/
/**************************************************************************/
void Adafruit_INA2xx::setAlertPolarity(INA2XX_AlertPolarity polarity) {
  /*Adafruit_I2CRegisterBits alert_polarity =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 12);
  alert_polarity.write(polarity);*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  diag_alrt &= ~(1 << 12);          // Clear APOL bit (12)
  diag_alrt |= (polarity << 12);    // Set new polarity
  write_register_16bit(INA2XX_REG_DIAGALRT, diag_alrt);
}
/**************************************************************************/
/*!
    @brief Reads the current alert latch setting
    @return The current bus alert latch setting
*/
/**************************************************************************/
INA2XX_AlertLatch Adafruit_INA2xx::getAlertLatch(void) {
  /*Adafruit_I2CRegisterBits alert_latch =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 15);
  return (INA2XX_AlertLatch)alert_latch.read();*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  return (INA2XX_AlertLatch)((diag_alrt >> 15) & 1); // Check ALATCH bit (15)
}
/**************************************************************************/
/*!
    @brief Sets Alert Latch Bit
    @param state
          The parameter which asserts the ALERT pin
*/
/**************************************************************************/
void Adafruit_INA2xx::setAlertLatch(INA2XX_AlertLatch state) {
  /*Adafruit_I2CRegisterBits alert_latch =
      Adafruit_I2CRegisterBits(Diag_Alert, 1, 15);
  alert_latch.write(state);*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  diag_alrt &= ~(1 << 15);          // Clear ALATCH bit (15)
  diag_alrt |= (state << 15);       // Set new state
  write_register_16bit(INA2XX_REG_DIAGALRT, diag_alrt);
}
/**************************************************************************/
/*!
    @brief Reads the alert function flags from DIAG_ALRT
    @return Bits that indicate alert flags
*/
/**************************************************************************/
uint16_t Adafruit_INA2xx::alertFunctionFlags(void) {
  /*Adafruit_I2CRegisterBits alert_flags =
      Adafruit_I2CRegisterBits(Diag_Alert, 12, 0);
  return alert_flags.read();*/
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  return diag_alrt & 0xFFF; // Return alert flags (bits 11:0)
}
/**
 * @brief reads 16 bit register
 * @return read
 */
int32_t Adafruit_INA2xx::read_register_16bit(uint8_t reg) {
  if(fd_ < 0) return -1;
  uint8_t write_buf[1] = {reg};
  if(write(fd_, write_buf, 1) != 1) {
    return -1;
  }
  uint8_t read_buf[2];
    if (read(fd_, read_buf, 2) != 2) {
        return -1;
    }
    // Combine MSB and LSB, convert from network byte order (big-endian)
    return ntohs(*(reinterpret_cast<uint16_t*>(read_buf)));
  
}

/**
 * @brief reads 24 bit register signed
 * @return signed 24 bit
 */
int32_t Adafruit_INA2xx::read_register_24bit_signed(uint8_t reg)
{
    if (fd_ < 0) return -1;
    uint8_t write_buf[1] = {reg};
    if (write(fd_, write_buf, 1) != 1) {
        return -1;
    }
    uint8_t read_buf[3];
    if (read(fd_, read_buf, 3) != 3) {
        return -1;
    }
    // Combine 3 bytes into a 32-bit integer (big-endian)
    int32_t value = (read_buf[0] << 16) | (read_buf[1] << 8) | read_buf[2];
    
    // Sign-extend from 24 bits to 32 bits
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    return value;
}

/**
 * @brief reads 24 bits from register
 * @return unsigned
 */
uint32_t Adafruit_INA2xx::read_register_24bit_unsigned(uint8_t reg)
{
    if (fd_ < 0) return -1;
    uint8_t write_buf[1] = {reg};
    if (write(fd_, write_buf, 1) != 1) {
        return -1;
    }
    uint8_t read_buf[3];
    if (read(fd_, read_buf, 3) != 3) {
        return -1;
    }
    // Combine 3 bytes into a 32-bit integer (big-endian)
    return (read_buf[0] << 16) | (read_buf[1] << 8) | read_buf[2];
}

/**
 * @brief writes 16 bits to register
 */
void Adafruit_INA2xx::write_register_16bit(uint8_t reg, uint16_t value)
{
    if (fd_ < 0) return;
    // Convert value to network byte order (big-endian)
    uint16_t network_value = htons(value);
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (network_value >> 8) & 0xFF; // MSB
    buf[2] = network_value & 0xFF;       // LSB
    
    if (write(fd_, buf, 3) != 3) {
        std::cerr << "Failed to write to register " << (int)reg << std::endl;
    }
}
}