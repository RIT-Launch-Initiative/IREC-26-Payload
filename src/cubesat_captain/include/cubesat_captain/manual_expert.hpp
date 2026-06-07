#pragma once
#include "cubesat_captain/expert.hpp"
#include "cubesat_msgs/action/extend_arm.hpp"
#include "cubesat_msgs/action/flip_servo_action.hpp"

#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/flip_servo.hpp"
#include "cubesat_msgs/msg/payload_orientation.hpp"
#include <utility>

namespace cubesat_captain {

class ManualExpert : public Expert {
  public:
    using ExtendArm = cubesat_msgs::action::ExtendArm;
    using GoalHandleExtendArm = rclcpp_action::ClientGoalHandle<ExtendArm>;
    using FlipServoAction = cubesat_msgs::action::FlipServoAction;
    using GoalHandleFlipServoAction = rclcpp_action::ClientGoalHandle<FlipServoAction>;

    ManualExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}
    ~ManualExpert() {}

    void enter_state() override;

    void arm_response_cb(GoalHandleExtendArm::SharedPtr);
    void arm_result_cb(const GoalHandleExtendArm::WrappedResult &);
    void arm_feedback_cb(GoalHandleExtendArm::SharedPtr,
                                       const std::shared_ptr<const ExtendArm::Feedback> feedback);

    void send_arm_to_target(const cubesat_msgs::action::ExtendArm::Goal &goal) override;
    void send_arm_to_target_and_come_back(const cubesat_msgs::action::ExtendArm::Goal &goal);
    void send_arm_to_target_and_come_back_with_photo(const cubesat_msgs::action::ExtendArm::Goal &goal);
};
} // namespace cubesat_captain
