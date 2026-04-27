#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

#include "sensor_reader/adxl375.hpp"
#include "sensor_reader/ina260.hpp"

class SensorReaderNode : public rclcpp::Node {
public:
  SensorReaderNode();

  ~SensorReaderNode() override;

private:
  void timeoutCallback();

  int readIntervalMs;

  std::string adxl375Topic;
  std::string adxl375I2cDev;
  uint8_t adxl375Addr;

  std::string ina260Topic;
  std::string ina260I2cDev;
  uint8_t ina260Addr;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr adxl375Publisher;
  Adxl375 adxl375;
  bool adxl375Initialized{false};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr ina260Publisher;
  Ina260 ina260;
  bool ina260Initialized{false};
};