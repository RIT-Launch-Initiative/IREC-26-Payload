#include "cubesat_captain/flipping_state.hpp"
#include "cubesat_captain/common.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include <array>

namespace flipping {
using AccelSample = cubesat_msgs::msg::AccelSample;
using Orientation = cubesat_msgs::msg::PayloadOrientation;

float dot(const cubesat_msgs::msg::AccelSample &lhs, const cubesat_msgs::msg::AccelSample &rhs) {
    return lhs.ax * rhs.ax + lhs.ay * rhs.ay + lhs.az * rhs.az;
}

struct FaceAndVector {
    uint8_t face;
    cubesat_msgs::msg::AccelSample vector;

    FaceAndConfidence to_confidence(const cubesat_msgs::msg::AccelSample &norm_sample) {
        return FaceAndConfidence{face, dot(norm_sample, vector)};
    }
};

cubesat_msgs::msg::AccelSample from_nums(float x, float y, float z) {
    cubesat_msgs::msg::AccelSample samp;
    samp.ax = x;
    samp.ay = y;
    samp.az = z;
    return samp;
}

const cubesat_msgs::msg::AccelSample deck = from_nums(0, 0, -1);
const cubesat_msgs::msg::AccelSample hull = from_nums(0, 0, 1);
const cubesat_msgs::msg::AccelSample starboard = from_nums(1, 0, 0);
const cubesat_msgs::msg::AccelSample port = from_nums(-1, 0, 0);
const cubesat_msgs::msg::AccelSample bow = from_nums(0, 1, 0);
const cubesat_msgs::msg::AccelSample stern = from_nums(0, -1, 0);

std::array<FaceAndVector, 6> faces{
    // clang-format off
    FaceAndVector{Orientation::SIDE_RECO_BAY, bow},          
    FaceAndVector{Orientation::SIDE_BASE, stern},
    FaceAndVector{Orientation::SIDE_BACKPLATE, hull},        
    FaceAndVector{Orientation::SIDE_TOP, deck},
    FaceAndVector{Orientation::SIDE_DOUBLE_WALL, starboard}, 
    FaceAndVector{Orientation::SIDE_TOP, port},
    // clang-format on
};

FaceAndConfidence which_side(const cubesat_msgs::msg::AccelSample &sample){
    auto norm_sample = normalize_accel(sample);
    std::array<FaceAndConfidence, 6> confidence{};
    for (size_t i = 0; i < 6; i++){
        confidence[i] = faces[i].to_confidence(norm_sample);
    }

    std::sort(confidence.begin(), confidence.end(), [](FaceAndConfidence a, FaceAndConfidence b){return a.confidence < b.confidence;});

    return confidence[5];
}

} // namespace flipping
