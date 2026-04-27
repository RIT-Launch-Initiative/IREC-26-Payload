#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cubesat_radio/radio_profile.hpp"
#include "cubesat_radio/radio_types.hpp"
#include "cubesat_radio/sx1262_radio.hpp"

namespace cubesat_radio {

class RadioNode : public rclcpp::Node {
public:
    explicit RadioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    RadioProfile loadProfile();
    RadioHardwareConfig loadHardwareConfig();

    std::unique_ptr<Sx1262Radio> radio_;
};

}  // namespace cubesat_radio
