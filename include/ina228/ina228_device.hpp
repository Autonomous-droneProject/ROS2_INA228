#ifndef TFLUNA_TFLUNA_DEVICE_HPP_
#define TFLUNA_TFLUNA_DEVICE_HPP_

#include "ina228/Adafruit_INA228.h"
#include "ina228/Adafruit_INA2xx.h"
#include "tca9548a/i2c_device.hpp"
#include "tca9548a/tca9548a.hpp"

namespace ina228 {
struct Config {
  INA2XX_MeasurementMode mode;
  INA2XX_ConversionTime voltageTime;
  INA2XX_ConversionTime currentTime;
  INA2XX_AveragingCount count;
};
class INA228Device : public tca9548a::I2CDevice {
 public:
  INA228Device();
  INA228Device(std::string i2c_bus, Config config_);
  virtual ~INA228Device() = default;
  bool initialize() override;
  bool configure() override;
  tca9548a::msg::SensorData read() override;

 private:
  ina228::Adafruit_INA228 sensor_;

  Config config_;

 private:
};
}; // namespace tfluna

#endif