#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cubesat_msgs/msg/accel_sample.hpp>
#include <cubesat_msgs/msg/gps_sample.hpp>
#include <cubesat_msgs/msg/power_sample.hpp>
#include <cubesat_msgs/srv/set_buzzer.hpp>

#include "cubesat_pi_io/gps_nmea_reader.hpp"
#include "cubesat_pi_io/ina260_driver.hpp"
#include "cubesat_pi_io/lis3dh_driver.hpp"
#include "cubesat_pi_io/buzzer_driver.hpp"

namespace cubesat_pi_io {

class PiIoNode : public rclcpp::Node {
  public:
    explicit PiIoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void onGpsTimer();
    void onPowerTimer();
    void onAccelTimer();
    void onBuzzerTimer();

    void setBuzzer(const std::shared_ptr<cubesat_msgs::srv::SetBuzzer::Request> request,
                  std::shared_ptr<cubesat_msgs::srv::SetBuzzer::Response> response);



    GpsNmeaReader gps;
    Ina260Driver ina;
    Lis3dhDriver lis;
    BuzzerDriver buzzer;

    rclcpp::Publisher<cubesat_msgs::msg::GpsSample>::SharedPtr gps_pub;
    rclcpp::Publisher<cubesat_msgs::msg::PowerSample>::SharedPtr power_pub;
    rclcpp::Publisher<cubesat_msgs::msg::AccelSample>::SharedPtr accel_pub;
    rclcpp::Service<cubesat_msgs::srv::SetBuzzer>::SharedPtr buzzer_service;

    rclcpp::TimerBase::SharedPtr gps_timer;
    rclcpp::TimerBase::SharedPtr power_timer;
    rclcpp::TimerBase::SharedPtr accel_timer;
    rclcpp::TimerBase::SharedPtr buzzer_timer;
};

} // namespace cubesat_pi_io
