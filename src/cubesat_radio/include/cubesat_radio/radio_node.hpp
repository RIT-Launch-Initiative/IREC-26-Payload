#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "cubesat_msgs/msg/radio_packet.hpp"
#include "cubesat_msgs/srv/send_radio_packet.hpp"

#include "cubesat_radio/radio_profile.hpp"
#include "cubesat_radio/radio_types.hpp"
#include "cubesat_radio/sx1262_radio.hpp"

namespace cubesat_radio {

class RadioNode : public rclcpp::Node {
public:
  explicit RadioNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RadioNode() override;

private:
  RadioProfile loadProfile();
  RadioHardwareConfig loadHardwareConfig();
  void receiveLoop();
  bool sendPacket(const std::vector<uint8_t> &data);
  void handleTxPacket(const cubesat_msgs::msg::RadioPacket::SharedPtr msg);
  void handleSendRadioPacket(
      const std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Request>
          &request,
      std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Response> response);

  std::unique_ptr<Sx1262Radio> radio;
  rclcpp::Publisher<cubesat_msgs::msg::RadioPacket>::SharedPtr rxPacketPub;
  rclcpp::Subscription<cubesat_msgs::msg::RadioPacket>::SharedPtr txPacketSub;
  rclcpp::Service<cubesat_msgs::srv::SendRadioPacket>::SharedPtr
      sendRadioPacketSrv;
  std::thread rxThread;
  std::atomic<bool> running{false};
  std::atomic<bool> txActive{false};
};

} // namespace cubesat_radio
