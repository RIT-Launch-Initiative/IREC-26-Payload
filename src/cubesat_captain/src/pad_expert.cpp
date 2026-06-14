#include "cubesat_captain/pad_expert.hpp"
#include "cubesat_captain/common.hpp"

namespace cubesat_captain {

void PadExpert::enter_state() {
    levers.set_primary_heartbeat(cubesat_msgs::msg::TelemetryType::FLIGHT_HEARTBEAT);
    levers.set_runcam_power(false);
    levers.status.clear_takeoff_time();
    has_boosted_ = false;
    avger.Fill(0);
}

void PadExpert::handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) {

    double mag = accel_magnitude(sample);
    avger.Feed(mag);

    has_boosted_ |= (avger.Avg() > levers.status.current_parameters.boost_threshold_mps2);
    if (has_boosted_) {
        RCLCPP_INFO(logger, "Detected boost: Avg: %f Current: %f,%f,%f  = %f", avger.Avg(), sample.ax, sample.ay, sample.az, mag);
        levers.goto_state(State::Flight);
    }
}

void PreboostExpert::handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) {
    pad_expert->handle_base_accel(sample);
}

void PreboostExpert::enter_state() {
    pad_expert->enter_state();
    levers.set_runcam_power(true);
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