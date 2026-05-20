#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cubesat_radio/radio_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cubesat_radio::RadioNode>());
    rclcpp::shutdown();
    return 0;
}
