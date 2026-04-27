#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cubesat_msgs/msg/accel_sample.hpp>
#include <cubesat_msgs/msg/gps_sample.hpp>
#include <cubesat_msgs/msg/power_sample.hpp>

#include "cubesat_pi_io/gps_nmea_reader.hpp"
#include "cubesat_pi_io/ina260_driver.hpp"
#include "cubesat_pi_io/lis3dh_driver.hpp"

namespace cubesat_pi_io {

class PiIoNode : public rclcpp::Node {
public:
  explicit PiIoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void onGpsTimer();
  void onPowerTimer();
  void onAccelTimer();

  GpsNmeaReader gps_;
  Ina260Driver  ina_;
  Lis3dhDriver  lis_;

  rclcpp::Publisher<cubesat_msgs::msg::GpsSample>::SharedPtr   gps_pub_;
  rclcpp::Publisher<cubesat_msgs::msg::PowerSample>::SharedPtr power_pub_;
  rclcpp::Publisher<cubesat_msgs::msg::AccelSample>::SharedPtr accel_pub_;

  rclcpp::TimerBase::SharedPtr gps_timer_;
  rclcpp::TimerBase::SharedPtr power_timer_;
  rclcpp::TimerBase::SharedPtr accel_timer_;
};

}  // namespace cubesat_pi_io
