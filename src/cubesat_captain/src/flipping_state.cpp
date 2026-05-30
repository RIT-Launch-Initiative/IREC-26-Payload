#include "cubesat_captain/flipping_state.hpp"
#include "cubesat_captain/common.hpp"
#include "cubesat_msgs/action/flip_servo_action.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/flip_servo.hpp"
#include <array>

namespace cubesat_captain {
using AccelSample = cubesat_msgs::msg::AccelSample;
using Orientation = cubesat_msgs::msg::PayloadOrientation;

float dot(const cubesat_msgs::msg::AccelSample &lhs, const cubesat_msgs::msg::AccelSample &rhs) {
    return lhs.ax * rhs.ax + lhs.ay * rhs.ay + lhs.az * rhs.az;
}

struct FaceAndVector {
    uint8_t face;
    cubesat_msgs::msg::AccelSample vector;

    FaceAndConfidence to_confidence(const cubesat_msgs::msg::AccelSample &norm_sample) {
        return FaceAndConfidence{face, dot(norm_sample, vector)};
    }
};

cubesat_msgs::msg::AccelSample from_nums(float x, float y, float z) {
    cubesat_msgs::msg::AccelSample samp;
    samp.ax = x;
    samp.ay = y;
    samp.az = z;
    return samp;
}

const cubesat_msgs::msg::AccelSample deck = from_nums(0, 0, -1);
const cubesat_msgs::msg::AccelSample hull = from_nums(0, 0, 1);
const cubesat_msgs::msg::AccelSample starboard = from_nums(1, 0, 0);
const cubesat_msgs::msg::AccelSample port = from_nums(-1, 0, 0);
const cubesat_msgs::msg::AccelSample bow = from_nums(0, 1, 0);
const cubesat_msgs::msg::AccelSample stern = from_nums(0, -1, 0);

std::array<FaceAndVector, 6> faces{
    // clang-format off
    FaceAndVector{Orientation::SIDE_RECO_BAY, bow},          
    FaceAndVector{Orientation::SIDE_BASE, stern},
    FaceAndVector{Orientation::SIDE_BACKPLATE, hull},        
    FaceAndVector{Orientation::SIDE_TOP, deck},
    FaceAndVector{Orientation::SIDE_DOUBLE_WALL, starboard}, 
    FaceAndVector{Orientation::SIDE_SINGLE_WALL, port},
    // clang-format on
};

FaceAndConfidence FlippingExpert::which_side(const cubesat_msgs::msg::AccelSample &sample) {
    auto norm_sample = normalize_accel(sample);
    std::array<FaceAndConfidence, 6> confidence{};
    for (size_t i = 0; i < 6; i++) {
        confidence[i] = faces[i].to_confidence(norm_sample);
    }

    std::sort(confidence.begin(), confidence.end(),
              [](FaceAndConfidence a, FaceAndConfidence b) { return a.confidence < b.confidence; });

    return confidence[5];
}

void FlippingExpert::start_flip(cubesat_msgs::msg::FlipServo &servo) {
    using namespace std::placeholders;

    auto goal = FlipServoAction::Goal();
    goal.servo_id = servo;
    goal.open_duration = 100;
    goal.openness = 255;
    goal.open_travel_duration = 200;
    goal.closedness = 0;
    goal.close_travel_duration = 120;

    auto send_goal_options = rclcpp_action::Client<FlipServoAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleFlipServoAction::SharedPtr future) {
        this->flip_response_cb(future);
    };
    send_goal_options.feedback_callback = std::bind(&FlippingExpert::flip_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FlippingExpert::flip_result_cb, this, _1);

    levers.flip_servo_action_client->async_send_goal(goal, send_goal_options);
}

void FlippingExpert::flip_finish() {
    FaceAndConfidence face = which_side(levers.status.last_base_accel);
    cubesat_msgs::msg::FlipServo servo{};
    if (face.side == cubesat_msgs::msg::PayloadOrientation::SIDE_DOUBLE_WALL) {
        RCLCPP_INFO(logger, "On Double wall");
        servo.id = cubesat_msgs::msg::FlipServo::FLIP_SERVO_3;
        start_flip(servo);
    } else if (face.side == cubesat_msgs::msg::PayloadOrientation::SIDE_SINGLE_WALL) {
        RCLCPP_INFO(logger, "On Single wall");
        servo.id = cubesat_msgs::msg::FlipServo::FLIP_SERVO_1;
        start_flip(servo);
    } else if (face.side == cubesat_msgs::msg::PayloadOrientation::SIDE_TOP) {
        RCLCPP_WARN(logger, "Got unlucky stuck on top. going to try again");
        servo.id = cubesat_msgs::msg::FlipServo::FLIP_SERVO_2;
        start_flip(servo);

    } else if (face.side == cubesat_msgs::msg::PayloadOrientation::SIDE_BACKPLATE) {
        RCLCPP_INFO(logger, "Got good and am upright");
        levers.goto_state(State::Unfolding);
    } else {
        RCLCPP_WARN(logger, "Very unlucky. Going to flail double wall");
        servo.id = cubesat_msgs::msg::FlipServo::FLIP_SERVO_1;
        start_flip(servo);
    }
}

void FlippingExpert::enter_state() {
    // todo check if this has been done before (possible if we were sent back to flipping)
    cubesat_msgs::msg::FlipServo servo{};
    servo.id = cubesat_msgs::msg::FlipServo::FLIP_SERVO_2;
    start_flip(servo);
}

void FlippingExpert::flip_response_cb(GoalHandleFlipServoAction::SharedPtr future) {
    RCLCPP_INFO(logger, "Response CB");
}
void FlippingExpert::flip_result_cb(const GoalHandleFlipServoAction::WrappedResult &result) {
    RCLCPP_INFO(logger, "Result CB");
    flip_finish();
    RCLCPP_INFO(logger, "Post Finish");
}
void FlippingExpert::flip_feedback_cb(GoalHandleFlipServoAction::SharedPtr,
                                      const std::shared_ptr<const FlipServoAction::Feedback> feedback) {
    RCLCPP_INFO(logger, "Feedbakc CB");
}

} // namespace cubesat_captain
