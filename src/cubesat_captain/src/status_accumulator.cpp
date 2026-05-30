#include "cubesat_captain/status_accumulator.hpp"

namespace cubesat_captain {

void StatusAccumulator::update_flight_state(const cubesat_msgs::msg::FlightState &state) { current_state = state; }

void StatusAccumulator::update_arm_status(const cubesat_msgs::msg::ArmStatus &status) { last_arm_status = status; }
void StatusAccumulator::update_gps_sample(const cubesat_msgs::msg::GpsSample &sample) {
    last_gps_sample = sample;
    if (sample.fix_type != 0) {
        last_good_gps_sample = sample;
    }
}
void StatusAccumulator::update_power_sample(const cubesat_msgs::msg::PowerSample &sample) {
    last_power_sample = sample;
}
void StatusAccumulator::update_base_accel(const cubesat_msgs::msg::AccelSample &sample) { last_base_accel = sample; }
State StatusAccumulator::active_state() { return (State)current_state.state; }

} // namespace cubesat_captain