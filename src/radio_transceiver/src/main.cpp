#include <rclcpp/rclcpp.hpp>

#include "radio_transceiver/RadioTransceiverNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadioTransceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
