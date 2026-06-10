#include "cubesat_captain/pad_expert.hpp"
#include "cubesat_captain/common.hpp"

namespace cubesat_captain {

constexpr double gravity_mps2 = 9.81;

void PadExpert::enter_state() {
    levers.set_primary_heartbeat(cubesat_msgs::msg::TelemetryType::FLIGHT_HEARTBEAT);
    levers.set_runcam_power(false);
    levers.status.clear_takeoff_time();
    clear_boost();
}

void PadExpert::handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) {

    double mag = accel_magnitude(sample);
    avger.Feed(mag);

    // boost_threshold_mps2 is the sustained acceleration in excess of the 1 g the
    // accelerometer reads while sitting still; raw magnitude on the pad is ~9.81
    has_boosted_ |= (avger.Avg() > gravity_mps2 + levers.status.current_parameters.boost_threshold_mps2);
}

void PreboostExpert::handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) {
    pad_expert->handle_base_accel(sample);
    if (pad_expert->has_boosted()) {
        RCLCPP_WARN(logger, "Boost detected, entering Flight");
        levers.goto_state(State::Flight);
    }
}

void PreboostExpert::enter_state() {
    levers.set_runcam_power(true);
    pad_expert->clear_boost();
}

} // namespace cubesat_captain

double accel_magnitude(const cubesat_msgs::msg::AccelSample &sample) {
    double mag_sqred = sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az;
    double mag = std::sqrt(mag_sqred);
    return mag;
}
cubesat_msgs::msg::AccelSample normalize_accel(const cubesat_msgs::msg::AccelSample &sample) {
    double mag = accel_magnitude(sample);
    cubesat_msgs::msg::AccelSample output{};
    output.ax = sample.ax / mag;
    output.ay = sample.ay / mag;
    output.az = sample.az / mag;
    return output;
}