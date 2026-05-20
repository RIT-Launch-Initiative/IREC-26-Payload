#include <rclcpp/rclcpp.hpp>

#include "cubesat_pi_io/pi_io_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cubesat_pi_io::PiIoNode>());
    rclcpp::shutdown();
    return 0;
}
