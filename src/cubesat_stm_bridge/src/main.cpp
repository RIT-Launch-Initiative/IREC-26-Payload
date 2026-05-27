#include <rclcpp/rclcpp.hpp>

#include "cubesat_stm_bridge/stm_bridge_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cubesat_stm_bridge::StmBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
