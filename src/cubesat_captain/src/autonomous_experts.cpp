#include "cubesat_captain/autonomous_experts.hpp"
#include <array>
#include <chrono>
#include <span>
#include <thread>

namespace cubesat_captain {
constexpr std::chrono::milliseconds camera_wait = std::chrono::milliseconds{2000};

static constexpr size_t allowed_per_side = 10;
static constexpr size_t num_allowed_to_ignore_stall = 2;

// really large deadband bc as long as we're reasonably close, we might as well try the next step
static int constexpr shoulder_deadband = 10;
static int constexpr elbow_deadband = 10;

bool ArmPose::isCloseEnoughTo(const ArmPose &other) const {
    return std::abs(shoulder_yaw - other.shoulder_yaw) < shoulder_deadband &&
           std::abs(shoulder_pitch - other.shoulder_pitch) < shoulder_deadband &&
           std::abs(elbow_pitch - other.elbow_pitch) < elbow_deadband;
}

ArmPose unfold_path[4] = {

    ArmPose{0, 90, -120, 127}, // folded up flight configuration
    ArmPose{0, 90, -60, 127}, // point up (cam pointed towards reco bay)
    ArmPose{0, 0, -60, -80},  // lift shoulder  (cam towards stern and a bit up)
    ArmPose{0, 0, -20, -127}, // straighten up (cam towards stern again, link 2 not all the way see the ground)
};

ArmPose panorama_path[12] = {
    ArmPose{0, 0, -20, -127},   // straighten up (cam towards stern again)
    ArmPose{0, 0, 20, 127},     // camera towards reco bay, shoulder pointing to reco bay
    ArmPose{45, 0, 20, 127},    // point off left shoulder between left and reco bay
    ArmPose{90, 0, 20, 127},    // point off left shoulder
    ArmPose{90, 0, -35, -127},  // flip over and look down. shoulder to the left, camera to the right
    ArmPose{45, 0, -35, -127},  // shoulder to the left/reco bay, camera to the back right
    ArmPose{0, 5, -40, -127},   // shoulder to the reco bay, camera off the back. lean forward a bit o make sure we dont
                                // hit the back plate
    ArmPose{-45, 5, -40, -127}, // shoulder to the right/reco bay, camera off the back left
    ArmPose{-90, 5, -40, -127}, // shoulder to the right, camera off the left
    ArmPose{-90, -5, 20, 110},  // shoulder to the right, camera off the left
    ArmPose{-45, -5, 20, 110},  // shoulder to the right/reco bay, camera off the right/reco bay
    ArmPose{0, 0, 10, 127},     // arm straight up, elbow slightly to reco bay, wrist towards reco bay
};

std::span<ArmPose> path_for_state(ArmState state) {
    if (state == ArmState::Unfolding) {
        return std::span<ArmPose>(unfold_path);
    } else {
        return std::span<ArmPose>(panorama_path);
    }
}

void ArmExpert::send_target(const ArmPose &target, bool ignore_stall) {
    using namespace std::placeholders;
    cubesat_msgs::action::ExtendArm::Goal goal;
    goal.shoulder_yaw = target.shoulder_yaw;
    goal.shoulder_pitch = target.shoulder_pitch;
    goal.elbow_pitch = target.elbow_pitch;
    goal.wrist_pitch = target.wrist_pitch;
    goal.ignore_stall = ignore_stall;

    auto send_goal_options = rclcpp_action::Client<ExtendArm>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](GoalHandleExtendArm::SharedPtr future) {
        this->arm_response_cb(future);
    };
    send_goal_options.feedback_callback = std::bind(&ArmExpert::arm_feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ArmExpert::arm_result_cb, this, _1);

    levers.extend_arm_client->async_send_goal(goal, send_goal_options);
}

void ArmExpert::finish_good() {
    if (for_state == ArmState::Unfolding) {
        levers.goto_state(State::AutoCamera);
    } else {
        levers.goto_state(State::ManualControl);
    }
}

bool ArmExpert::stillInState() {
    if (for_state == ArmState::Panoramaing) {
        return (levers.status.active_state() == State::AutoCamera);
    } else {
        return (levers.status.active_state() == State::Unfolding);
    }
    return false;
}
void ArmExpert::decide_next() {
    std::span<ArmPose> path = path_for_state(for_state);

    // just incase we didn't check last time
    if (path_index >= path.size() - 1) {
        RCLCPP_INFO(logger, "Finished arm path. Going to next state");
        finish_good();
        return;
    }

    if (attempts_for_this_side > allowed_per_side) {
        RCLCPP_INFO(logger, "Over number of tries allowed, going to emergency mode");
        levers.goto_state(State::Emergency);
        return;
    }

    const cubesat_msgs::msg::ArmStatus arm = levers.status.last_arm_status;
    const ArmPose current{(int8_t)arm.shoulder_yaw_deg, (int8_t)arm.shoulder_pitch_deg, (int8_t)arm.elbow_angle_deg,
                          (int8_t)arm.wrist_angle_deg};

    if (current.isCloseEnoughTo(path[path_index])) {
        // take picture
        RCLCPP_INFO(logger, "Arm step  %ld/%ld success. Taking pic and continuing", path_index + 1, path.size());
        levers.take_picture(0, 1280, 0, 800, 640, 2);
        std::thread([this]() {
            std::this_thread::sleep_for(camera_wait);
            RCLCPP_INFO(logger, "Assuming camera took picture, continuing");
            if (!stillInState()) {
                return;
            }
            path_index++;
            attempts_for_this_side = 0;
            std::span<ArmPose> path = path_for_state(for_state);
            if (path_index > path.size()) {
                finish_good();
                return;
            }
            send_target(path[path_index], false); // never ignore stall on first attempt

            attempts_for_this_side++;
        }).detach();
    } else {
        RCLCPP_INFO(logger, "Arm step  %ld/%ld fail. Trying again", path_index + 1, path.size());
        bool ignore_stall = attempts_for_this_side > allowed_per_side - num_allowed_to_ignore_stall;
        send_target(path[path_index], ignore_stall);
        attempts_for_this_side++;
    }
}
void ArmExpert::enter_state() {
    const cubesat_msgs::msg::ArmStatus arm = levers.status.last_arm_status;
    const ArmPose current{(int8_t)arm.shoulder_yaw_deg, (int8_t)arm.shoulder_pitch_deg, (int8_t)arm.elbow_angle_deg,
                          (int8_t)arm.wrist_angle_deg};
    levers.set_runcam_power(true);

    if (!current.isCloseEnoughTo(path_for_state(for_state)[0])) {
        RCLCPP_ERROR(logger, "Not anywhere close to starting point. Lowk an emergency");
        levers.goto_state(State::Emergency);
        return;
    }

    levers.set_primary_heartbeat(cubesat_msgs::msg::TelemetryType::LANDED_HEARTBEAT);
    RCLCPP_INFO(logger, "Starting arm mode for state %d", (int)for_state);
    path_index = 0;
    attempts_for_this_side = 1;
    send_target(path_for_state(for_state)[0], false);
}

void ArmExpert::arm_response_cb(GoalHandleExtendArm::SharedPtr goal) {
    if (!goal) {
        RCLCPP_WARN(logger, "Arm extend request rejected");
        decide_next();
    }
}
void ArmExpert::arm_result_cb(const GoalHandleExtendArm::WrappedResult &) { decide_next(); }
void ArmExpert::arm_feedback_cb(GoalHandleExtendArm::SharedPtr, const std::shared_ptr<const ExtendArm::Feedback>) {}
} // namespace cubesat_captain