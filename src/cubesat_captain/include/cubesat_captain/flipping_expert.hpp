#pragma once
#include "cubesat_captain/expert.hpp"
#include "cubesat_msgs/action/flip_servo_action.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/flip_servo.hpp"
#include "cubesat_msgs/msg/payload_orientation.hpp"
#include <utility>

namespace cubesat_captain {

struct FaceAndConfidence {
    uint8_t side;
    float confidence; // -1 is opposite, 0 is orthogonal, 1 is right on
};

class FlippingExpert : public Expert {
  public:
    using FlipServoAction = cubesat_msgs::action::FlipServoAction;
    using GoalHandleFlipServoAction = rclcpp_action::ClientGoalHandle<FlipServoAction>;

    FlippingExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}
    ~FlippingExpert() {}

    void enter_state() override;

    void start_flip(cubesat_msgs::msg::FlipServo &servo);
    void flip_finish();

    void flip_response_cb(GoalHandleFlipServoAction::SharedPtr future);
    void flip_result_cb(const GoalHandleFlipServoAction::WrappedResult &result);
    void flip_feedback_cb(GoalHandleFlipServoAction::SharedPtr,
                          const std::shared_ptr<const FlipServoAction::Feedback> feedback);

    static FaceAndConfidence which_side(const cubesat_msgs::msg::AccelSample &sample);
};

} // namespace cubesat_captain
