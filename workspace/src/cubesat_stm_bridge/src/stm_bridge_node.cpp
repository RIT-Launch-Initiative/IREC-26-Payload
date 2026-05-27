#include "cubesat_stm_bridge/stm_bridge_node.hpp"

#include <chrono>
#include <string>

namespace cubesat_stm_bridge {

namespace {

std::chrono::nanoseconds periodFromHz(double hz) {
    if (hz <= 0.0) {
        return std::chrono::seconds(1);
    }
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

} // namespace

StmBridgeNode::StmBridgeNode(const rclcpp::NodeOptions &options) : rclcpp::Node("stm_bridge_node", options) {
    // Shared (consumed by other nodes declared here so launch can pass it freely).
    declare_parameter<std::string>("flight_dir", "");

    RCLCPP_INFO(get_logger(), "Stm Bridge Node started");

    const auto status_hz = declare_parameter<double>("status_poll_hz", 50.0);

    const auto spi_device = declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
    const auto spi_bus_hz = declare_parameter<int>("spi_freq_hz", 2000000);

    arm_pub = create_publisher<cubesat_msgs::msg::ArmStatus>("stm/arm_status", 10);

    if (!crashout.open(spi_device, spi_bus_hz)) {
        RCLCPP_ERROR(get_logger(), "Crashout STM creation failed: spidev='%s' spi_hz=%ld", spi_device.c_str(),
                     spi_bus_hz);
    }
    // if (!ina.open(ina_dev, static_cast<uint8_t>(ina_addr))) {
    //     RCLCPP_ERROR(get_logger(), "INA260 open failed: device='%s' addr=0x%02X", ina_dev.c_str(), ina_addr);
    // }
    // if (!lis.open(lis_dev, static_cast<uint8_t>(lis_addr))) {
    //     RCLCPP_ERROR(get_logger(), "LIS3DH open failed: device='%s' addr=0x%02X", lis_dev.c_str(), lis_addr);
    // } else if (!lis.configure(static_cast<uint16_t>(lis_rate), static_cast<uint8_t>(lis_range))) {
    //     RCLCPP_ERROR(get_logger(),
    //                  "LIS3DH configure failed: rate=%d Hz range=±%dg (allowed "
    //                  "rates: 1/10/25/50/100/200/400, ranges: 2/4/8/16)",
    //                  lis_rate, lis_range);
    // }

    status_timer = create_wall_timer(periodFromHz(status_hz), [this] { onStatusTimer(); });
    // power_timer = create_wall_timer(periodFromHz(ina_hz), [this] { onPowerTimer(); });
    // accel_timer = create_wall_timer(periodFromHz(lis_hz), [this] { onAccelTimer(); });
}

void StmBridgeNode::FillArmStatusFlags(StmBridge::StatusWord word, cubesat_msgs::msg::ArmStatus &status) {
    using namespace StmBridge;
    status.booted = CheckStatusBit(word, StatusBit_Booted);
    status.moving_arm = CheckStatusBit(word, StatusBit_MovingArm);
    status.moving_flip_servo1 = CheckStatusBit(word, StatusBit_MovingFlipServo1);
    status.moving_flip_servo2 = CheckStatusBit(word, StatusBit_MovingFlipServo2);
    status.moving_flip_servo3 = CheckStatusBit(word, StatusBit_MovingFlipServo3);
    status.arm_move_failed = CheckStatusBit(word, StatusBit_MovingArmFailed);
    status.wrist_servo_en = CheckStatusBit(word, StatusBit_WristServoEn);
    status.flip_servo_en = CheckStatusBit(word, StatusBit_FlipServoEn);
    status.motor_en = CheckStatusBit(word, StatusBit_MotorEn);
    status.overtemp = CheckStatusBit(word, StatusBit_Overtemp);
}

void StmBridgeNode::onStatusTimer() {
    using namespace StmBridge;
    auto maybe_status = crashout.getStatus();
    if (!maybe_status.has_value()) {
        RCLCPP_WARN(get_logger(), "Failed to get status from STM");
        return;
    }
    StmBridge::Status arm_status = *maybe_status;
    cubesat_msgs::msg::ArmStatus pub_status;
    FillArmStatusFlags(arm_status.status_word, pub_status);
    pub_status.shoulder_yaw_deg = arm_status.pose.shoulder_yaw;
    pub_status.shoulder_pitch_deg = arm_status.pose.shoulder_pitch;
    pub_status.elbow_angle_deg = arm_status.pose.elbow_pitch;
    pub_status.wrist_angle_deg = arm_status.pose.wrist_pitch;
    last_status = pub_status;
    
}

} // namespace cubesat_stm_bridge
