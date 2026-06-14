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

static constexpr size_t imu_every = 50;

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
    imu_sub = create_subscription<cubesat_msgs::msg::AccelSample>(
        "pi/lis3dh", 10, std::bind(&StmBridgeNode::handle_imu, this, std::placeholders::_1));

    using namespace std::placeholders;

    this->flip_action_server_ = rclcpp_action::create_server<FlipServoAction>(
        this, "/stm/flip_servo", std::bind(&StmBridgeNode::handle_flip_goal, this, _1, _2),
        std::bind(&StmBridgeNode::handle_flip_cancel, this, _1),
        std::bind(&StmBridgeNode::handle_flip_accepted, this, _1));

    this->arm_action_server_ = rclcpp_action::create_server<ExtendArm>(
        this, "/stm/move_arm", std::bind(&StmBridgeNode::handle_arm_goal, this, _1, _2),
        std::bind(&StmBridgeNode::handle_arm_cancel, this, _1),
        std::bind(&StmBridgeNode::handle_arm_accepted, this, _1));

    this->hold_service = create_service<cubesat_msgs::srv::HoldShut>(
        "/stm/hold_shut", std::bind(&StmBridgeNode::holdShut, this, std::placeholders::_1, std::placeholders::_2));

    this->jog_service = create_service<cubesat_msgs::srv::JogMotor>(
        "/stm/jog_motor", std::bind(&StmBridgeNode::jogMotor, this, std::placeholders::_1, std::placeholders::_2));

    this->zero_arm_service = create_service<cubesat_msgs::srv::ZeroArm>(
        "/stm/zero_arm", std::bind(&StmBridgeNode::zeroArm, this, std::placeholders::_1, std::placeholders::_2));

    status_timer = create_wall_timer(periodFromHz(status_hz), [this] { onStatusTimer(); });
}

void StmBridgeNode::FillArmStatusFlags(StmBridge::StatusWord word, cubesat_msgs::msg::ArmStatus &status) {
    using namespace StmBridge;
    uint8_t state_u8 = (uint8_t)((word >> 1) & 0b111);
    status.booted = CheckStatusBit(word, StatusBit_Booted);
    cubesat_msgs::msg::ArmState arm_state{};
    arm_state.state = state_u8;

    status.state = arm_state;
    status.booted = CheckStatusBit(word, StatusBit_Booted);
    status.arm_move_failed = CheckStatusBit(word, StatusBit_MovingArmFailed);
    status.wrist_servo_en = CheckStatusBit(word, StatusBit_WristServoEn);
    status.flip_servo_en = CheckStatusBit(word, StatusBit_FlipServoEn);
    status.motor_en = CheckStatusBit(word, StatusBit_MotorEn);
    status.cant_trust_imu_link = CheckStatusBit(word, StatusBitCantTrustImuLink);
    status.encoders_not_updating = CheckStatusBit(word, StatusBitEncodersNotUpdating);
}

void StmBridgeNode::onStatusTimer() {
    using namespace StmBridge;
    auto maybe_status = crashout.setBaseImuAndReturnStatus(normed_v16(last_pi_imu));
    if (!maybe_status.has_value()) {
        RCLCPP_WARN(get_logger(), "Failed to get status from STM");
    }

    StmBridge::Status arm_status = *maybe_status;
    cubesat_msgs::msg::ArmStatus pub_status;
    FillArmStatusFlags(arm_status.status_word, pub_status);
    pub_status.stamp = now();
    pub_status.shoulder_yaw_deg = arm_status.pose.shoulder_yaw;
    pub_status.shoulder_pitch_deg = arm_status.pose.shoulder_pitch;
    pub_status.elbow_angle_deg = arm_status.pose.elbow_pitch;
    pub_status.wrist_angle_deg = arm_status.pose.wrist_pitch;
    pub_status.last_link1_accel = last_l1_imu;
    pub_status.last_link2_accel = last_l2_imu;

    last_status = pub_status;
    arm_pub->publish(pub_status);

    counter++;
    if (counter % imu_every == 1) {
        auto value = crashout.getLink1IMU();
        if (value) {
            last_l1_imu.ax = value->x;
            last_l1_imu.ay = value->y;
            last_l1_imu.az = value->z;
        }
    } else if (counter % imu_every == 2) {
        auto value = crashout.getLink2IMU();
        if (value) {
            last_l2_imu.ax = value->x;
            last_l2_imu.ay = value->y;
            last_l2_imu.az = value->z;
        }
    }

    switch (active_mode) {
    case BridgeMode::Idle:
        break;
    case BridgeMode::MovingServo1:
        tickServo(StmBridge::Servo1);
        break;
    case BridgeMode::MovingServo2:
        tickServo(StmBridge::Servo2);
        break;
    case BridgeMode::MovingServo3:
        tickServo(StmBridge::Servo3);
        break;
    case BridgeMode::MovingArm:
        tickArm();
        break;
    case BridgeMode::Holding:
        // don't need to do anything
        break;
    default:
        RCLCPP_WARN(get_logger(), "Unimplemented mode tick");
    }
}

void StmBridgeNode::tickServo(StmBridge::FlipServo servoid) {
    if (flip_servo_action_handle == nullptr) {
        RCLCPP_WARN(get_logger(), "Was in servo mode but servo action goal handle was null!!");
        active_mode = BridgeMode::Idle;
        return;
    }
    using ArmState = cubesat_msgs::msg::ArmState;
    bool still_moving = false;
    switch (servoid) {
    case StmBridge::Servo1:
        still_moving = last_status.state.state == ArmState::STATE_SERVO1_MOVING;
        break;
    case StmBridge::Servo2:
        still_moving = last_status.state.state == ArmState::STATE_SERVO2_MOVING;
        break;
    case StmBridge::Servo3:
        still_moving = last_status.state.state == ArmState::STATE_SERVO3_MOVING;
        break;
    }
    if (now() < flip_should_show_progress_time) {
        // don't cancel just bc we asked before it realized we commanded it
        still_moving = true;
    }

    bool overtime = now() > flip_timeout_time;
    if (!still_moving || overtime) {
        RCLCPP_INFO(get_logger(), "Finished Servo Movement. Overtime %s", overtime ? "yes" : "no");
        // say end (if not cancelled, success)
        auto result = std::make_shared<FlipServoAction::Result>();
        result->success = !overtime;
        if (overtime) {
            flip_servo_action_handle->abort(result);
        } else {
            flip_servo_action_handle->succeed(result);
        }
        active_mode = BridgeMode::Idle;
        flip_servo_action_handle = nullptr;
        return;
    }

    if (flip_servo_action_handle->is_canceling()) {
        crashout.stopMovement();
        RCLCPP_INFO(get_logger(), "Cancelled Servo Movement");
        auto result = std::make_shared<FlipServoAction::Result>();
        result->success = false;
        flip_servo_action_handle->canceled(result);
        active_mode = BridgeMode::Idle;
        flip_servo_action_handle = nullptr;
        return;
    }

    auto feedback = std::make_shared<FlipServoAction::Feedback>();
    feedback->progress = 0.5;
    flip_servo_action_handle->publish_feedback(feedback);
}

void StmBridgeNode::tickArm() {
    if (arm_action_handle == nullptr) {
        RCLCPP_WARN(get_logger(), "Was in arm mode but arm action goal handle was null!!");
        active_mode = BridgeMode::Idle;
        return;
    }
    using ArmState = cubesat_msgs::msg::ArmState;
    bool still_moving = last_status.state.state == ArmState::STATE_ARM_MOVING;

    if (now() < arm_should_show_progress_time) {
        // don't cancel just bc we asked before it realized we commanded it
        still_moving = true;
    }

    bool overtime = now() > arm_timeout_time;
    if (!still_moving || overtime) {
        RCLCPP_INFO(get_logger(), "Finished Arm Movement. Overtime %s, movement failed: %s", overtime ? "yes" : "no",
                    last_status.arm_move_failed ? "yes" : "no");
        // say end (if not cancelled, success)
        auto result = std::make_shared<ExtendArm::Result>();
        result->success = !overtime && !last_status.arm_move_failed;
        result->overcurrent = last_status.arm_move_failed;
        if (overtime) {
            arm_action_handle->abort(result);
        } else {
            arm_action_handle->succeed(result);
        }
        active_mode = BridgeMode::Idle;
        arm_action_handle = nullptr;
        return;
    }

    if (arm_action_handle->is_canceling()) {
        crashout.stopMovement();
        RCLCPP_INFO(get_logger(), "Cancelled Arm Movement");
        auto result = std::make_shared<ExtendArm::Result>();
        result->success = false;
        arm_action_handle->canceled(result);
        active_mode = BridgeMode::Idle;
        arm_action_handle = nullptr;
        return;
    }

    auto feedback = std::make_shared<ExtendArm::Feedback>();
    feedback->arm_status = last_status;
    arm_action_handle->publish_feedback(feedback);
}

void StmBridgeNode::holdShut(const std::shared_ptr<cubesat_msgs::srv::HoldShut::Request> request,
                             std::shared_ptr<cubesat_msgs::srv::HoldShut::Response> response) {
    RCLCPP_INFO(get_logger(), "Request to hold=%d", (int)request->should_hold);
    if (request->should_hold) {
        if (active_mode == BridgeMode::Idle) {
            // start
            crashout.startHold();
            active_mode = BridgeMode::Holding;
            response->success = true;
        } else {
            // decline
            response->success = false;
        }
    } else {
        if (active_mode == BridgeMode::Idle) {
            // already there, immediate success
            response->success = true;
        } else if (active_mode == BridgeMode::Holding) {
            // do it
            RCLCPP_INFO(get_logger(), "Stopping Hold");
            crashout.stopMovement();
            active_mode = BridgeMode::Idle;
            response->success = true;
        }
    }
}

void StmBridgeNode::jogMotor(const std::shared_ptr<cubesat_msgs::srv::JogMotor::Request> request,
                             std::shared_ptr<cubesat_msgs::srv::JogMotor::Response> response) {
    RCLCPP_INFO(get_logger(), "Request to jog motor idx=%d iterations = %d ms at %f V", (int)request->motor,
                (int)request->milliseconds, request->voltage);
    if (active_mode == BridgeMode::Idle) {
        // start
        crashout.setJogMovement(request->motor, request->milliseconds / 10, (int16_t)(request->voltage * 1000));
        crashout.startJogMovement();
        response->success = true;
    } else {
        RCLCPP_WARN(get_logger(), "Not jogging motors because not idle. Was: %d", (int)active_mode);
        // decline
        response->success = false;
    }
}

void StmBridgeNode::zeroArm(const std::shared_ptr<cubesat_msgs::srv::ZeroArm::Request> request,
                            std::shared_ptr<cubesat_msgs::srv::ZeroArm::Response> response) {
    RCLCPP_INFO(get_logger(), "Request to zero arm to (%d, %d, %d, %d)", request->shoulder_yaw, request->shoulder_pitch,
                request->elbow_angle, request->wrist_angle);

    if (active_mode == BridgeMode::Idle) {
        // start
        crashout.setPoseEst(
            {request->shoulder_yaw, request->shoulder_pitch, request->elbow_angle, request->wrist_angle});
        response->success = true;
    } else {
        RCLCPP_WARN(get_logger(), "Not zeroing arm because not idle. Was: %d", (int)active_mode);
        // decline
        response->success = false;
    }
}

rclcpp_action::GoalResponse StmBridgeNode::handle_flip_goal(const rclcpp_action::GoalUUID &uuid,
                                                            std::shared_ptr<const FlipServoAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request to flip servo%d. Open to %d in %d ms. Hold for %d ms. Close to %d in %d ms",
                ((int)(goal->servo_id.id) + 1), goal->openness, (goal->open_travel_duration * 10),
                (goal->open_duration * 10), goal->closedness, (goal->close_travel_duration * 10));
    (void)uuid;
    if (active_mode == BridgeMode::Idle) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(get_logger(), "Rejecting flip request because not idle");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse
StmBridgeNode::handle_flip_cancel(const std::shared_ptr<GoalHandleFlipServo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel flip");
    (void)goal_handle;
    // always accept cancels, handled in tickServo / tickArm
    return rclcpp_action::CancelResponse::ACCEPT;
}

StmBridge::FlipServo rosToStm(cubesat_msgs::msg::FlipServo servo) {
    switch (servo.id) {
    case 0:
        return StmBridge::FlipServo::Servo1;
    case 1:
        return StmBridge::FlipServo::Servo2;
    case 2:
        return StmBridge::FlipServo::Servo3;
    }
    return StmBridge::FlipServo::Servo1;
}

StmBridgeNode::BridgeMode servoToMode(cubesat_msgs::msg::FlipServo servo) {
    switch (servo.id) {
    case 0:
        return StmBridgeNode::BridgeMode::MovingServo1;
    case 1:
        return StmBridgeNode::BridgeMode::MovingServo2;
    case 2:
        return StmBridgeNode::BridgeMode::MovingServo3;
    }
    return StmBridgeNode::BridgeMode::Idle;
}

void StmBridgeNode::handle_flip_accepted(const std::shared_ptr<GoalHandleFlipServo> goal_handle) {
    using namespace std::placeholders;
    const FlipServoAction::Goal goal = *goal_handle->get_goal();
    const StmBridge::FlipServoMotion motion{
        .open_duration = goal.open_duration,
        .openness = goal.openness,
        .open_travel_duration = goal.open_travel_duration,
        .closedness = goal.closedness,
        .close_travel_duration = goal.close_travel_duration,
    };
    crashout.setServoMotion(rosToStm(goal.servo_id), motion);
    active_mode = servoToMode(goal.servo_id);
    flip_servo_action_handle = goal_handle;

    flip_start_time = now();
    // 30 ms to see response
    flip_should_show_progress_time = flip_start_time + rclcpp::Duration(0, 30 * 1000 * 1000);
    // timeout if its > 2x the amount of time we expect
    uint32_t duration_ms = motion.total_duration() * 10;
    uint64_t duration_ns = ((uint64_t)duration_ms * 1000 * 1000) * 2;
    flip_timeout_time = flip_start_time + rclcpp::Duration(duration_ns / 1000000000, duration_ns % 1000000000);
    RCLCPP_INFO(get_logger(), "Timeout %ld ns. Starting at %ld, waiting for %ld, timeout at %ld", duration_ns,
                flip_start_time.nanoseconds(), flip_should_show_progress_time.nanoseconds(),
                flip_timeout_time.nanoseconds());

    crashout.startServoMovement(rosToStm(goal.servo_id));
}

rclcpp_action::GoalResponse StmBridgeNode::handle_arm_goal(const rclcpp_action::GoalUUID &uuid,
                                                           std::shared_ptr<const ExtendArm::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request to move arm. goto (%d, %d, %d, %d)", goal->shoulder_yaw,
                goal->shoulder_pitch, goal->elbow_pitch, goal->wrist_pitch);
    (void)uuid;
    if (active_mode == BridgeMode::Idle) {
        active_mode = BridgeMode::MovingArm;
        RCLCPP_WARN(get_logger(), "Accepting arm move request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(get_logger(), "Rejecting arm move request because not idle");
        return rclcpp_action::GoalResponse::REJECT;
    }
}
rclcpp_action::CancelResponse StmBridgeNode::handle_arm_cancel(const std::shared_ptr<GoalHandleExtendArm> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel arm");
    (void)goal_handle;
    // always accept cancels, handled in tickServo / tickArm
    return rclcpp_action::CancelResponse::ACCEPT;
}

void StmBridgeNode::handle_arm_accepted(const std::shared_ptr<GoalHandleExtendArm> goal_handle) {
    const ExtendArm::Goal goal = *goal_handle->get_goal();
    const StmBridge::ArmPose pose{
        .shoulder_yaw = goal.shoulder_yaw,
        .shoulder_pitch = goal.shoulder_pitch,
        .elbow_pitch = goal.elbow_pitch,
        .wrist_pitch = goal.wrist_pitch,
    };
    crashout.setBaseImuAndReturnStatus(normed_v16(last_pi_imu));

    crashout.setArmTarget(pose);
    arm_action_handle = goal_handle;
    arm_start_time = now();

    arm_should_show_progress_time = arm_start_time + rclcpp::Duration(0, 30 * 1000 * 1000);
    // timeout if its > 2x the amount of time we expect
    uint32_t duration_ms = 15000;
    uint64_t duration_ns = ((uint64_t)duration_ms * 1000 * 1000) * 2;
    arm_timeout_time = arm_start_time + rclcpp::Duration(duration_ns / 1000000000, duration_ns % 1000000000);
    RCLCPP_INFO(get_logger(), "Timeout %ld ns. Starting at %ld, waiting for %ld, timeout at %ld", duration_ns,
                arm_start_time.nanoseconds(), arm_should_show_progress_time.nanoseconds(),
                arm_timeout_time.nanoseconds());

    crashout.startArmMovement(goal.ignore_stall);
}

void StmBridgeNode::handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample) { last_pi_imu = *sample; }
StmBridge::Vec3_16 StmBridgeNode::normed_v16(const cubesat_msgs::msg::AccelSample &sample) {
    float normsqred = sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az;
    float norm = std::sqrt(normsqred);
    if (std::abs(norm) < 0.001) {
        return {0, 0, 0};
    }
    float nx = sample.ax / norm;
    float ny = sample.ay / norm;
    float nz = sample.az / norm;
    return {(int16_t)(nx * 32767), (int16_t)(ny * 32767), (int16_t)(nz * 32767)};
}

} // namespace cubesat_stm_bridge
