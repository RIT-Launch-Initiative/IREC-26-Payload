#include "cubesat_captain/pad_state.hpp"
#include "cubesat_captain/common.hpp"
namespace pad {

static bool has_boosted_ = false;

// at 50 hz, 12 samples = 0.24 seconds
CMovingAverage<double, 12> avger{0.0};

void feed_boost_detect(const cubesat_msgs::msg::AccelSample &sample, double threshold) {
    double mag = accel_magnitude(sample);
    avger.Feed(mag);

    has_boosted_ |= (avger.Avg() > threshold);
}
bool has_boosted() { return has_boosted_; }
void fake_boost_detect() { has_boosted_ |= true; }

} // namespace pad

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