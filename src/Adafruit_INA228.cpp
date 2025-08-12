/*!
 * @file Adafruit_INA228.cpp
 *
 * @section ina228_intro Introduction
 *
 * I2C Driver for the INA228 I2C Current and Power sensor
 *
 * This is a library for the Adafruit INA228 breakout:
 * http://www.adafruit.com/products/5832
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products from
 * Adafruit!
 *
 * @section ina228_dependencies Dependencies
 *
 * This library now uses low-level Linux I2C calls.
 *
 * @section ina228_author Author
 *
 * Bryan Siepert for Adafruit Industries
 *
 * @section ina228_license License
 *
 * BSD (see license.txt)
 *
 * @section ina228_history HISTORY
 *
 * v1.0 - First release
 */

#include "ina228/Adafruit_INA228.h"
#include <unistd.h> // For read/write

namespace ina228 {

/*!
 * @brief  Instantiates a new INA228 class.
 * @param i2c_bus The path to the I2C bus (e.g., "/dev/i2c-1")
 * @param i2c_addr The I2C address of the sensor
 */
Adafruit_INA228::Adafruit_INA228(std::string i2c_bus, uint8_t i2c_addr) : Adafruit_INA2xx(i2c_bus, i2c_addr) {}

/*!
 * @brief  Sets up the HW and validates the device ID.
 * @param  skipReset
 * When set to true, will omit resetting all INA228 registers to
 * their default values. Default: false.
 * @return True if initialization was successful, otherwise false.
 */
bool Adafruit_INA228::begin(bool skipReset) {
  // Call the base class begin(), which opens the I2C port and checks the
  // manufacturer ID.
  if (!Adafruit_INA2xx::begin(skipReset)) {
    return false;
  }

  // After the base begin, read the device ID and make sure it's an INA228.
  // The device ID is the top 12 bits of the DVC_UID register.
  uint16_t device_id_raw = read_register_16bit(INA2XX_REG_DVC_UID);
  _device_id = device_id_raw >> 4; // Shift right by 4 to get the 12-bit ID

  if (_device_id != INA228_DEVICE_ID) {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief Resets the hardware. This calls the base class reset.
*/
/**************************************************************************/
void Adafruit_INA228::reset(void) {
  // Perform base class reset
  Adafruit_INA2xx::reset();
}

/**************************************************************************/
/*!
    @brief Updates the shunt calibration value to the INA228 register.
*/
/**************************************************************************/
void Adafruit_INA228::_updateShuntCalRegister() {
  float scale = 1.0;
  // If ADC Range is set to the lower range (+/- 40.96mV), the shunt cal
  // must be scaled by 4.
  if (getADCRange()) {
    scale = 4.0;
  }
  
  // Per datasheet formula: SHUNT_CAL = 13107.2 x 10^6 x CURRENT_LSB x R_SHUNT
  // The scale factor is applied for the lower ADC range.
  uint16_t shunt_cal = (uint16_t)(13107.2 * 1000000.0 * _current_lsb * _shunt_res * scale);

  write_register_16bit(INA2XX_REG_SHUNTCAL, shunt_cal);
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Energy register.
    @return The current Energy calculation in Joules
*/
/**************************************************************************/
double Adafruit_INA228::readEnergy(void) {
  uint8_t write_buf[1] = {INA228_REG_ENERGY};
  if (write(fd_, write_buf, 1) != 1) {
    return -1; // Indicate error
  }
  uint8_t read_buf[5];
  if (read(fd_, read_buf, 5) != 5) {
    return -1; // Indicate error
  }

  uint64_t energy_raw = 0;
  for (int i = 0; i < 5; i++) {
    energy_raw = (energy_raw << 8) | read_buf[i];
  }
  
  // Per datasheet formula: Energy (Joules) = ENERGY_REG * 16 * POWER_LSB
  // where POWER_LSB = 3.2 * CURRENT_LSB
  double power_lsb = 3.2 * _current_lsb;
  return (double)energy_raw * 16.0 * power_lsb;
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Charge register.
    @return The current Charge calculation in Coulombs
*/
/**************************************************************************/
double Adafruit_INA228::readCharge(void) {
  uint8_t write_buf[1] = {INA228_REG_CHARGE};
  if (write(fd_, write_buf, 1) != 1) {
    return -1; // Indicate error
  }
  uint8_t read_buf[5];
  if (read(fd_, read_buf, 5) != 5) {
    return -1; // Indicate error
  }

  // Convert 40-bit two's complement value to a 64-bit signed integer
  int64_t charge_raw = 0;
  for (int i = 0; i < 5; i++) {
    charge_raw = (charge_raw << 8) | read_buf[i];
  }

  // Handle sign extension for 40-bit two's complement
  if (charge_raw & ((int64_t)1 << 39)) {
    charge_raw |= 0xFFFFFF0000000000; // Sign extend to 64 bits
  }

  // Per datasheet formula: Charge (Coulombs) = CHARGE_REG * CURRENT_LSB
  return (double)charge_raw * _current_lsb;
}

/**************************************************************************/
/*!
    @brief Returns the current alert type.
    @return The current alert type configured in the DIAG_ALRT register.
*/
/**************************************************************************/
INA228_AlertType Adafruit_INA228::getAlertType(void) {
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  // Alert type is stored in bits 13:8
  return (INA228_AlertType)((diag_alrt >> 8) & 0x3F); // 0x3F is a mask for 6 bits
}

/**************************************************************************/
/*!
    @brief Sets a new alert type.
    @param alert The new alert type to be set.
*/
/**************************************************************************/
void Adafruit_INA228::setAlertType(INA228_AlertType alert) {
  uint16_t diag_alrt = read_register_16bit(INA2XX_REG_DIAGALRT);
  diag_alrt &= ~(0x3F << 8);    // Clear the 6 alert type bits (13:8)
  diag_alrt |= (alert << 8);    // Set the new alert type
  write_register_16bit(INA2XX_REG_DIAGALRT, diag_alrt);
}

/**************************************************************************/
/*!
    @brief Resets the energy and charge accumulators.
*/
/**************************************************************************/
void Adafruit_INA228::resetAccumulators(void) {
  uint16_t config_val = read_register_16bit(INA2XX_REG_CONFIG);
  config_val |= (1 << 14); // Set the RSTACC bit (14)
  write_register_16bit(INA2XX_REG_CONFIG, config_val);
}

/**************************************************************************/
/*!
    @brief Reads the die temperature with the INA228-specific conversion factor.
    @return The current die temp in deg C.
*/
/**************************************************************************/
float Adafruit_INA228::readDieTemp(void) {
  // This function is identical to the base class version for INA228.
  // It is kept for clarity and potential future specialization.
  return Adafruit_INA2xx::readDieTemp();
}

/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Bus Voltage register
           using INA228-specific conversion factor.
    @return The current bus voltage measurement in V.
*/
/**************************************************************************/
float Adafruit_INA228::readBusVoltage(void) {
  // This function is identical to the base class version for INA228.
  return Adafruit_INA2xx::readBusVoltage();
}

/**************************************************************************/
/*!
    @brief Sets the shunt calibration by resistor for INA228.
    @param shunt_res Resistance of the shunt in ohms (floating point).
    @param max_current Maximum expected current in A (floating point).
*/
/**************************************************************************/
void Adafruit_INA228::setShunt(float shunt_res, float max_current) {
  _shunt_res = shunt_res;
  // INA228 uses a resolution of 19 bits for current calculation.
  _current_lsb = max_current / (float)(1UL << 19);
  _updateShuntCalRegister();
}

} // namespace ina228