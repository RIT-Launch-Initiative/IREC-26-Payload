#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "cubesat_msgs/msg/radio_packet.hpp"

#include "cubesat_radio/radio_profile.hpp"
#include "cubesat_radio/radio_types.hpp"
#include "cubesat_radio/sx1262_radio.hpp"

namespace cubesat_radio {

class RadioNode : public rclcpp::Node {
public:
    explicit RadioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RadioNode() override;

private:
    RadioProfile loadProfile();
    RadioHardwareConfig loadHardwareConfig();
    void receiveLoop();

    std::unique_ptr<Sx1262Radio> radio;
    rclcpp::Publisher<cubesat_msgs::msg::RadioPacket>::SharedPtr rxPacketPub;
    std::thread rxThread;
    std::atomic<bool> running{false};
};

}  // namespace cubesat_radio
