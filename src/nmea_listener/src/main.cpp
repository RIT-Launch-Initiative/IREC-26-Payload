#include "nmea_listener/NmeaListenerNode.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NmeaListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
