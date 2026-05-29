#pragma once

#include "cubesat_captain/status_accumulator.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/arm_status.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/gps_sample.hpp"
#include "cubesat_msgs/msg/power_sample.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cubesat_captain {
struct Levers {
    StatusAccumulator &status;
    // things an expert can do
    void goto_state(State state);
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

  protected:
    rclcpp::Logger logger;
    Levers &levers;
};



} // namespace cubesat_captain