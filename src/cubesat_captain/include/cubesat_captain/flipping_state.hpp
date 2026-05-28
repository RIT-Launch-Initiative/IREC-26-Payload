#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/payload_orientation.hpp"
#include <utility>

namespace flipping {
struct FaceAndConfidence {
    uint8_t side;
    float confidence; // -1 is opposite, 0 is orthogonal, 1 is right on
};
FaceAndConfidence which_side(const cubesat_msgs::msg::AccelSample &sample);
} // namespace flipping
