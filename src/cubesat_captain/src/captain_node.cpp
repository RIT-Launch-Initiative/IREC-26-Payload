#include "cubesat_captain/captain_node.hpp"
#include "cubesat_captain/flipping_state.hpp"
#include "cubesat_captain/pad_state.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace cubesat_captain {

namespace {

std::chrono::nanoseconds periodFromHz(double hz) {
    if (hz <= 0.0) {
        return std::chrono::seconds(1);
    }
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

} // namespace

CaptainNode::CaptainNode(const rclcpp::NodeOptions &options) : rclcpp::Node("captain_node", options) {
    flight_dir = declare_parameter<std::string>("flight_dir", "~/unconfigured_flight_dir");
    load_startup_parameters();

    RCLCPP_INFO(get_logger(), "Captain Node started: Flight Time: %f", status.current_parameters.flight_time_s);
    RCLCPP_INFO(get_logger(), "Pad HB: %f s, Flight HB %f s, Landed HB %f s", status.current_parameters.pad_heartbeat_s,
                status.current_parameters.flight_heartbeat_s, status.current_parameters.landed_heartbeat_s);

    if (!std::filesystem::exists(flight_dir)) {
        RCLCPP_ERROR(get_logger(), "flight_dir doesnt exist. Creating");
        std::filesystem::create_directory(flight_dir);
    }
    if (!std::filesystem::is_directory(flight_dir)) {
        RCLCPP_ERROR(get_logger(), "flight_dir isn't a directory??");
    }

    state_pub = create_publisher<cubesat_msgs::msg::FlightState>("pi/flight_state", 10);
    imu_sub = create_subscription<cubesat_msgs::msg::AccelSample>(
        "pi/lis3dh", 10, std::bind(&CaptainNode::handle_imu, this, std::placeholders::_1));

    enter_flipping();
}

void CaptainNode::load_startup_parameters() {
    status.current_parameters.flight_time_s = declare_parameter<double>("flight_time_s", 100);
    status.current_parameters.pad_heartbeat_s = declare_parameter<double>("pad_heartbeat_s", 0.05);
    status.current_parameters.flight_heartbeat_s = declare_parameter<double>("flight_heartbeat_s", 0.2);
    status.current_parameters.landed_heartbeat_s = declare_parameter<double>("landed_heartbeat_s", 0.2);
    status.current_parameters.boost_threshold_mps2 = declare_parameter<double>("boost_threshold_mps2", 7);
}

void CaptainNode::change_internal_state(State state) {
    cubesat_msgs::msg::FlightState msg;
    msg.stamp = now();
    msg.state = (uint8_t)state;
    status.update_flight_state(msg);
    state_pub->publish(msg);
}
void CaptainNode::enter_pad() {
    change_internal_state(State::Pad);
    RCLCPP_INFO(get_logger(), "In Pad State");
}
void CaptainNode::enter_preboost() {
    change_internal_state(State::Preboost);
    RCLCPP_INFO(get_logger(), "In Preboost State");
    // turn on cameras
    // start timer for return to pad (maybe)
}
void CaptainNode::enter_flight() { change_internal_state(State::Flight); }
void CaptainNode::enter_flipping() { change_internal_state(State::Flipping); }
void CaptainNode::enter_unfolding() { change_internal_state(State::Unfolding); }
void CaptainNode::enter_auto_camera() { change_internal_state(State::AutoCamera); }
void CaptainNode::enter_manual() { change_internal_state(State::ManualControl); }
void CaptainNode::enter_emergency() { change_internal_state(State::Emergency); }

void CaptainNode::flag_for_new_flight_dir() { std::ofstream(flight_dir + "/new_dir_please.flag").close(); }

void CaptainNode::restart_system() {
    RCLCPP_WARN(get_logger(), "Restarting self (except not actually bc not implemented)");
}

void CaptainNode::handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample) {
    status.update_base_accel(*sample);
    switch (status.active_state()) {
    case State::Pad:
        pad::feed_boost_detect(*sample, status.current_parameters.boost_threshold_mps2);
        if (pad::has_boosted()) {
            enter_flight();
        }
        break;
    case State::Preboost:
        pad::feed_boost_detect(*sample, status.current_parameters.boost_threshold_mps2);
        if (pad::has_boosted()) {
            enter_flight();
        }
        break;
    case State::Flight:
        break;
    case State::Flipping: {
        flipping::FaceAndConfidence face = flipping::which_side(*sample);
        RCLCPP_INFO(get_logger(), "On Face %d with confidence %f", face.side, face.confidence);
    } break;
    case State::Unfolding:
        break;
    case State::AutoCamera:
        break;
    case State::ManualControl:
        break;
    case State::Emergency:
        break;
    };
}

} // namespace cubesat_captain
