#pragma once

#include "crashout_stm.hpp"
#include "stm_data.hpp"
#include <cubesat_msgs/msg/arm_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cubesat_stm_bridge {

class StmBridgeNode : public rclcpp::Node {
  public:
    explicit StmBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    rclcpp::Publisher<cubesat_msgs::msg::ArmStatus>::SharedPtr arm_pub;
    // rclcpp::Publisher<cubesat_msgs::msg::PowerSample>::SharedPtr power_pub;
    // rclcpp::Publisher<cubesat_msgs::msg::AccelSample>::SharedPtr accel_pub;

    rclcpp::TimerBase::SharedPtr status_timer;
    // rclcpp::TimerBase::SharedPtr power_timer;
    // rclcpp::TimerBase::SharedPtr accel_timer;

    void onStatusTimer();

    void FillArmStatusFlags(StmBridge::StatusWord word, cubesat_msgs::msg::ArmStatus &status);

  private:
    StmBridge::CrashoutSTM crashout;
    cubesat_msgs::msg::ArmStatus last_status;
};

} // namespace cubesat_stm_bridge
