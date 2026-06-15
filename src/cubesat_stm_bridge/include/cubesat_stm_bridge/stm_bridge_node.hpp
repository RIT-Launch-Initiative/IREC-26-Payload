#pragma once

#include "crashout_stm.hpp"
#include "stm_data.hpp"
#include "cubesat_msgs/srv/hold_shut.hpp"
#include "cubesat_msgs/srv/jog_motor.hpp"
#include "cubesat_msgs/srv/zero_arm.hpp"

#include <cubesat_msgs/action/flip_servo_action.hpp>
#include <cubesat_msgs/action/extend_arm.hpp>

#include <cubesat_msgs/msg/arm_state.hpp>
#include <cubesat_msgs/msg/arm_status.hpp>
#include <cubesat_msgs/msg/accel_sample.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cubesat_stm_bridge {

class StmBridgeNode : public rclcpp::Node {
  public:
    using FlipServoAction = cubesat_msgs::action::FlipServoAction;
    using GoalHandleFlipServo = rclcpp_action::ServerGoalHandle<FlipServoAction>;

    using ExtendArm = cubesat_msgs::action::ExtendArm;
    using GoalHandleExtendArm = rclcpp_action::ServerGoalHandle<ExtendArm>;


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


    void attemptRestart();
    void onStatusTimer();
    void tickServo(StmBridge::FlipServo servoid);
    void tickArm();

    void holdShut(const std::shared_ptr<cubesat_msgs::srv::HoldShut::Request> request,
                  std::shared_ptr<cubesat_msgs::srv::HoldShut::Response> response);

    void jogMotor(const std::shared_ptr<cubesat_msgs::srv::JogMotor::Request> request,
                  std::shared_ptr<cubesat_msgs::srv::JogMotor::Response> response);

    void zeroArm(const std::shared_ptr<cubesat_msgs::srv::ZeroArm::Request> request,
                  std::shared_ptr<cubesat_msgs::srv::ZeroArm::Response> response);

    void FillArmStatusFlags(StmBridge::StatusWord word, cubesat_msgs::msg::ArmStatus &status);

    rclcpp_action::GoalResponse handle_flip_goal(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const FlipServoAction::Goal> goal);
    rclcpp_action::CancelResponse handle_flip_cancel(const std::shared_ptr<GoalHandleFlipServo> goal_handle);
    void handle_flip_accepted(const std::shared_ptr<GoalHandleFlipServo> goal_handle);



    rclcpp_action::GoalResponse handle_arm_goal(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const ExtendArm::Goal> goal);
    rclcpp_action::CancelResponse handle_arm_cancel(const std::shared_ptr<GoalHandleExtendArm> goal_handle);

    void handle_arm_accepted(const std::shared_ptr<GoalHandleExtendArm> goal_handle);



  private:
    rclcpp::Subscription<cubesat_msgs::msg::AccelSample>::SharedPtr imu_sub;
    void handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample);
    StmBridge::Vec3_16 normed_v16(const cubesat_msgs::msg::AccelSample &sample);

    BridgeMode active_mode{BridgeMode::Idle};


    size_t counter = 0;

    rclcpp_action::Server<FlipServoAction>::SharedPtr flip_action_server_;
    std::shared_ptr<GoalHandleFlipServo> flip_servo_action_handle{nullptr};
    rclcpp::Time flip_start_time{};
    rclcpp::Time flip_should_show_progress_time{};
    rclcpp::Time flip_timeout_time{};

    rclcpp_action::Server<ExtendArm>::SharedPtr arm_action_server_;
    std::shared_ptr<GoalHandleExtendArm> arm_action_handle{nullptr};
    rclcpp::Time arm_start_time{};
    rclcpp::Time arm_should_show_progress_time{};
    rclcpp::Time arm_timeout_time{};

    rclcpp::Service<cubesat_msgs::srv::ZeroArm>::SharedPtr zero_arm_service;
    rclcpp::Service<cubesat_msgs::srv::HoldShut>::SharedPtr hold_service;
    rclcpp::Service<cubesat_msgs::srv::JogMotor>::SharedPtr jog_service;

    StmBridge::CrashoutSTM crashout;
    cubesat_msgs::msg::ArmStatus last_status;
    cubesat_msgs::msg::AccelSample last_pi_imu;
    cubesat_msgs::msg::AccelSample last_l1_imu;
    cubesat_msgs::msg::AccelSample last_l2_imu;
};

} // namespace cubesat_stm_bridge
