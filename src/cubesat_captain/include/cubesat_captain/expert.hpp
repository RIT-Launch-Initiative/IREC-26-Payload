#pragma once

#include "cubesat_captain/status_accumulator.hpp"

#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/arm_status.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/gps_sample.hpp"
#include "cubesat_msgs/msg/power_sample.hpp"
#include "cubesat_msgs/msg/telemetry_type.hpp"

#include "cubesat_msgs/srv/hold_shut.hpp"
#include "cubesat_msgs/srv/jog_motor.hpp"
#include "cubesat_msgs/srv/zero_arm.hpp"

#include "cubesat_msgs/action/extend_arm.hpp"
#include "cubesat_msgs/action/flip_servo_action.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace cubesat_captain {
struct Levers {
    StatusAccumulator &status;

    rclcpp::Client<cubesat_msgs::srv::JogMotor>::SharedPtr jog_motor_client; // /stm/jog_motor
    rclcpp::Client<cubesat_msgs::srv::HoldShut>::SharedPtr hold_shut_client; // /stm/hold_shut
    rclcpp::Client<cubesat_msgs::srv::ZeroArm>::SharedPtr zero_arm_client;   // /stm/zero_arm

    rclcpp_action::Client<cubesat_msgs::action::ExtendArm>::SharedPtr extend_arm_client;              // stm/move_arm
    rclcpp_action::Client<cubesat_msgs::action::FlipServoAction>::SharedPtr flip_servo_action_client; // /stm/flip_servo
    // things an expert can do
    std::function<void(State state)> goto_state;
    std::function<void(cubesat_msgs::msg::TelemetryType::_telem_id_type type)> set_primary_heartbeat;
};

class Expert {
  public:
    Expert(rclcpp::Logger logger, Levers &levers) : logger(logger), levers(levers) {}
    virtual void enter_state() {}
    virtual void exit_state() {}

    virtual void handle_base_accel([[maybe_unused]] const cubesat_msgs::msg::AccelSample &sample) {}
    virtual void handle_arm_status([[maybe_unused]] const cubesat_msgs::msg::ArmStatus &status) {}
    virtual void handle_gps_sample([[maybe_unused]] const cubesat_msgs::msg::GpsSample &sample) {}
    virtual void handle_power_sample([[maybe_unused]] const cubesat_msgs::msg::PowerSample &sample) {}

    virtual void send_arm_to_target([[maybe_unused]] const cubesat_msgs::action::ExtendArm::Goal &goal) {
        RCLCPP_WARN(logger, "Ignoring request to send arm to target");
    }
    virtual void send_arm_to_target_and_come_back([[maybe_unused]] const cubesat_msgs::action::ExtendArm::Goal &goal) {
        RCLCPP_WARN(logger, "Ignoring request to send arm to target and come back");
    }
    virtual void
    send_arm_to_target_and_come_back_with_photo([[maybe_unused]] const cubesat_msgs::action::ExtendArm::Goal &goal) {
        RCLCPP_WARN(logger, "Ignoring request to send arm to target and come back with image");
    }

  protected:
    rclcpp::Logger logger;
    Levers &levers;
};

} // namespace cubesat_captain