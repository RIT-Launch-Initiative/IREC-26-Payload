#include <rclcpp/rclcpp.hpp>

#include "cubesat_captain/captain_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cubesat_captain::CaptainNode>());
    rclcpp::shutdown();
    return 0;
}
