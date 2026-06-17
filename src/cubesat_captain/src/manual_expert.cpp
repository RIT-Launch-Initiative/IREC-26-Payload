#include "cubesat_captain/manual_expert.hpp"
namespace cubesat_captain {
void ManualExpert::send_arm_to_target(const cubesat_msgs::action::ExtendArm::Goal &goal) {
    using namespace std::placeholders;

    RCLCPP_INFO(logger, "Received request to send arm to target: %d, %d, %d, %d", goal.shoulder_yaw,
                goal.shoulder_pitch, goal.elbow_pitch, goal.wrist_pitch);

    take_photo_with_arm_movement = false;
    return_to_tx_point_after_arm_movement = false;

    auto send_goal_options = rclcpp_action::Client<ExtendArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleExtendArm::SharedPtr goal) {
        if (!goal) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Extend arm goal was rejected by server");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Extend arm goal accepted by server");
        }
    };
    send_goal_options.feedback_callback = std::bind(&ManualExpert::arm_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ManualExpert::arm_result_cb, this, _1);

    levers.extend_arm_client->async_send_goal(goal, send_goal_options);
}

cubesat_msgs::action::ExtendArm::Goal tx_pose(int8_t current_yaw) {
    cubesat_msgs::action::ExtendArm::Goal goal;
    goal.shoulder_yaw = current_yaw;
    goal.shoulder_pitch = 0;
    goal.elbow_pitch = 0;
    goal.wrist_pitch = 0;
    return goal;
}

void ManualExpert::enter_state() { levers.set_primary_heartbeat(cubesat_msgs::msg::TelemetryType::LANDED_HEARTBEAT); }

void ManualExpert::send_arm_to_target_and_come_back(const cubesat_msgs::action::ExtendArm::Goal &goal) {
    using namespace std::placeholders;
    RCLCPP_INFO(logger, "Received request to send arm to target and come back: %d, %d, %d, %d", goal.shoulder_yaw,
                goal.shoulder_pitch, goal.elbow_pitch, goal.wrist_pitch);

    take_photo_with_arm_movement = false;
    return_to_tx_point_after_arm_movement = true;

    auto send_goal_options = rclcpp_action::Client<ExtendArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleExtendArm::SharedPtr goal) {
        if (!goal) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Extend arm goal and come back was rejected by server");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Extend arm goal and come back accepted by server");
        }
    };
    send_goal_options.feedback_callback = std::bind(&ManualExpert::arm_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ManualExpert::arm_result_cb, this, _1);

    levers.extend_arm_client->async_send_goal(goal, send_goal_options);
}
void ManualExpert::send_arm_to_target_and_come_back_with_photo(const cubesat_msgs::action::ExtendArm::Goal &goal) {
    using namespace std::placeholders;

    RCLCPP_INFO(logger, "Received request to send arm to target and come back with image: %d, %d, %d, %d",
                goal.shoulder_yaw, goal.shoulder_pitch, goal.elbow_pitch, goal.wrist_pitch);

    take_photo_with_arm_movement = true;
    return_to_tx_point_after_arm_movement = true;

    auto send_goal_options = rclcpp_action::Client<ExtendArm>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](GoalHandleExtendArm::SharedPtr goal) {
        if (!goal) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Extend arm goal and come back was rejected by server");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Extend arm goal and come back accepted by server");
        }
    };
    send_goal_options.feedback_callback = std::bind(&ManualExpert::arm_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ManualExpert::arm_result_cb, this, _1);

    levers.extend_arm_client->async_send_goal(goal, send_goal_options);
}

void ManualExpert::arm_response_cb(GoalHandleExtendArm::SharedPtr) { RCLCPP_INFO(logger, "Arm Response CB"); }

constexpr std::chrono::milliseconds camera_wait = std::chrono::milliseconds{2000};

void ManualExpert::arm_result_cb(const GoalHandleExtendArm::WrappedResult &) {
    RCLCPP_INFO(logger, "Manual Arm Result CB");
    RCLCPP_INFO(logger, "Arm Post Finish");
    if (take_photo_with_arm_movement) {
    }

    if (return_to_tx_point_after_arm_movement) {
        // wait 2 seconds, return

        std::thread([this]() {
            std::this_thread::sleep_for(camera_wait);
            RCLCPP_INFO(logger, "Assuming camera took picture, continuing");
            if (levers.status.active_state() != State::ManualControl) {
                return;
            }
            send_arm_to_target(tx_pose(levers.status.last_arm_status.shoulder_yaw_deg));
        }).detach();

    } else {
        // reset for next time
        return_to_tx_point_after_arm_movement = true;
        take_photo_with_arm_movement = true;
    }
}
void ManualExpert::arm_feedback_cb(GoalHandleExtendArm::SharedPtr,
                                   const std::shared_ptr<const ExtendArm::Feedback> feedback) {
    RCLCPP_INFO(logger, "Arm feedback CB: %f %f %f %f", feedback->arm_status.shoulder_yaw_deg,
                feedback->arm_status.shoulder_pitch_deg, feedback->arm_status.elbow_angle_deg,
                feedback->arm_status.wrist_angle_deg);
}

void ManualExpert::flip_response_cb(GoalHandleFlipServoAction::SharedPtr future) {
    RCLCPP_INFO(logger, "Manual Flip Response CB");
}
void ManualExpert::flip_result_cb(const GoalHandleFlipServoAction::WrappedResult &result) {
    RCLCPP_INFO(logger, "Manual Flip Response CB");
}
void ManualExpert::flip_feedback_cb(GoalHandleFlipServoAction::SharedPtr,
                                    const std::shared_ptr<const FlipServoAction::Feedback> feedback) {}

void ManualExpert::execute_servo_motion(const cubesat_msgs::action::FlipServoAction::Goal &goal) {
    using namespace std::placeholders;

    RCLCPP_INFO(logger, "Received request for manual servo #%d", goal.servo_id.id);

    auto send_goal_options = rclcpp_action::Client<FlipServoAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleFlipServoAction::SharedPtr goal) {
        if (!goal) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Flip Servo goal was rejected by server");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Flip servo goal accepted by server");
        }
    };
    send_goal_options.feedback_callback = std::bind(&ManualExpert::flip_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ManualExpert::flip_result_cb, this, _1);

    levers.flip_servo_action_client->async_send_goal(goal, send_goal_options);
}

} // namespace cubesat_captain