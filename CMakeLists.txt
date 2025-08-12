cmake_minimum_required(VERSION 3.8)
project(ina228)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tca9548a REQUIRED)

include_directories(include)

add_library(ina228_driver SHARED
  src/Adafruit_INA2xx.cpp
  src/Adafruit_INA228.cpp
  src/ina228_device.cpp
)

# Use ament_target_dependencies for external ROS packages
ament_target_dependencies(ina228_driver
  "rclcpp"
  "pluginlib"
  "tca9548a"
)

pluginlib_export_plugin_description_file(ina228_driver ina228_plugins.xml)

install(TARGETS ina228_driver
  EXPORT ina228_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(ina228_driver_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  pluginlib
  tca9548a
)

ament_package()