#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <string>
#include <thread>

#include "arm_msgs/msg/gps_status.hpp"

class NmeaListenerNode final : public rclcpp::Node {
public:
  explicit NmeaListenerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~NmeaListenerNode() override;

private:
  void readerLoop();
  int openSerial(const std::string &device, int baud);

private:
  struct CurrentState {
    double latitude{0.0};
    double longitude{0.0};
    uint8_t fix_type{0};
    uint8_t satellites_visible{0};
    uint32_t timestamp{0};
  };

  std::string port;
  int baud;
  std::string rawNmeaTopic;
  std::string gpsStatusTopic;
  int maxLineLen;

  rclcpp::Publisher<arm_msgs::msg::GpsStatus>::SharedPtr gpsStatusPublisher;

  CurrentState currentState;
  std::mutex currentStateMutex;

  int fd{-1};
  std::atomic<bool> running{false};
  std::thread readerThread;

  rclcpp::TimerBase::SharedPtr timer;

  void handleRawNmeaLine(const std::string &line);

  void publishGpsStatusCallback();
};
