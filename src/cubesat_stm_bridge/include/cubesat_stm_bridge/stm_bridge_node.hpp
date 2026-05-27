#pragma once

#include "crashout_stm.hpp"
#include "cubesat_msgs/srv/hold_shut.hpp"
#include "stm_data.hpp"
#include <cubesat_msgs/action/flip_servo_action.hpp>
#include <cubesat_msgs/msg/arm_state.hpp>
#include <cubesat_msgs/msg/arm_status.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cubesat_stm_bridge {

class StmBridgeNode : public rclcpp::Node {
  public:
    using FlipServoAction = cubesat_msgs::action::FlipServoAction;
    using GoalHandleFlipServo = rclcpp_action::ServerGoalHandle<FlipServoAction>;

    enum class BridgeMode {
        Idle,
        MovingServo1,
        MovingServo2,
        MovingServo3,
        MovingArm,
        Holding,
    };

    explicit StmBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    rclcpp::TimerBase::SharedPtr status_timer;
    rclcpp::Publisher<cubesat_msgs::msg::ArmStatus>::SharedPtr arm_pub;

    void onStatusTimer();
    void tickServo(StmBridge::FlipServo servoid);
    void tickArm();

    void holdShut(const std::shared_ptr<cubesat_msgs::srv::HoldShut::Request> request,
                  std::shared_ptr<cubesat_msgs::srv::HoldShut::Response> response);

    void FillArmStatusFlags(StmBridge::StatusWord word, cubesat_msgs::msg::ArmStatus &status);

    rclcpp_action::GoalResponse handle_flip_goal(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const FlipServoAction::Goal> goal);
    rclcpp_action::CancelResponse handle_flip_cancel(const std::shared_ptr<GoalHandleFlipServo> goal_handle);
    void handle_flip_accepted(const std::shared_ptr<GoalHandleFlipServo> goal_handle);

  private:
    BridgeMode active_mode{BridgeMode::Idle};
    std::shared_ptr<GoalHandleFlipServo> flip_servo_action_handle{nullptr};
    rclcpp::Time flip_start_time{};
    rclcpp::Time flip_should_show_progress_time{};
    rclcpp::Time flip_timeout_time{};

    rclcpp_action::Server<FlipServoAction>::SharedPtr flip_action_server_;
    rclcpp::Service<cubesat_msgs::srv::HoldShut>::SharedPtr hold_service;

    StmBridge::CrashoutSTM crashout;
    cubesat_msgs::msg::ArmStatus last_status;
};

} // namespace cubesat_stm_bridge
