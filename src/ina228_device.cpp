#include "ina228/ina228_device.hpp"

namespace ina228 {
INA228Device::INA228Device(std::string i2c_bus, Config config)
    : sensor_(i2c_bus), config_(config) {}

bool INA228Device::initialize() {
  return sensor_.begin();
}

bool INA228Device::configure() {
  sensor_.setAveragingCount(config_.count);
  sensor_.setVoltageConversionTime(config_.voltageTime);
  sensor_.setCurrentConversionTime(config_.currentTime);
  sensor_.setMode(config_.mode);

  return true;
}
tca9548a::msg::SensorData INA228Device::read() {
  tca9548a::msg::SensorData message;
  float current = sensor_.getCurrent_mA();
  float busv = sensor_.getBusVoltage_V();
  float shunt = sensor_.getShuntVoltage_mV();
  float power = sensor_.getPower_mW();
  float energy = sensor_.readEnergy();
  float charge = sensor_.readCharge();
  float temp = sensor_.readDieTemp();

    message.header.stamp = rclcpp::Clock().now();
    message.device_name = "ina228_sensor";

    diagnostic_msgs::msg::KeyValue current_kv;
    current_kv.key = "current_mA";
    current_kv.value = std::to_string(current);
    message.values.push_back(current_kv);

    diagnostic_msgs::msg::KeyValue busv_kv;
    busv_kv.key = "bus_V";
    busv_kv.value = std::to_string(busv);
    message.values.push_back(busv_kv);

    diagnostic_msgs::msg::KeyValue shunt_kv;
    shunt_kv.key = "shunt_mV";
    shunt_kv.value = std::to_string(shunt);
    message.values.push_back(shunt_kv);

    diagnostic_msgs::msg::KeyValue power_kv;
    power_kv.key = "power_mW";
    power_kv.value = std::to_string(power);
    message.values.push_back(power_kv);

    diagnostic_msgs::msg::KeyValue energy_kv;
    energy_kv.key = "energy";
    energy_kv.value = std::to_string(energy);
    message.values.push_back(energy_kv);

    diagnostic_msgs::msg::KeyValue charge_kv;
    charge_kv.key = "charge";
    charge_kv.value = std::to_string(charge);
    message.values.push_back(charge_kv);

    diagnostic_msgs::msg::KeyValue temp_kv;
    temp_kv.key = "temp";
    temp_kv.value = std::to_string(temp);
    message.values.push_back(temp_kv);
  return message;
}
} // namespace ina228