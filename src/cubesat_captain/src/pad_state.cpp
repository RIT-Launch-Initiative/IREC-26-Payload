#include "cubesat_captain/pad_state.hpp"
#include "cubesat_captain/common.hpp"

namespace cubesat_captain {

void PadExpert::handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) {

    double mag = accel_magnitude(sample);
    avger.Feed(mag);

    has_boosted_ |= (avger.Avg() > levers.status.current_parameters.boost_threshold_mps2);
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