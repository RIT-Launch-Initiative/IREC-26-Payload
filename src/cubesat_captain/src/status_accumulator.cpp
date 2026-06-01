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
State StatusAccumulator::active_state()const { return (State)current_state.state; }

bool StatusAccumulator::has_gps() const { return last_gps_sample.fix_type != 0; }
void StatusAccumulator::last_good_gps_position(float *lat, float *lon, float *alt) const {
    
    if (has_gps()){
        *lat = last_gps_sample.latitude;
        *lon = last_gps_sample.longitude;
        *alt = last_gps_sample.altitude_m;
    } else {
        *lat = last_good_gps_sample.latitude;
        *lon = last_good_gps_sample.longitude;
        *alt = last_good_gps_sample.altitude_m;
    }
}

} // namespace cubesat_captain