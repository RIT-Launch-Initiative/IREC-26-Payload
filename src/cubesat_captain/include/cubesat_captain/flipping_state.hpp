#pragma once
#include "cubesat_captain/expert.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/payload_orientation.hpp"
#include <utility>

namespace cubesat_captain {

struct FaceAndConfidence {
    uint8_t side;
    float confidence; // -1 is opposite, 0 is orthogonal, 1 is right on
};

class FlippingExpert : public Expert {
  public:
    FlippingExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}
    ~FlippingExpert() {}

    void handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) override {
        FaceAndConfidence face = which_side(sample);
        RCLCPP_INFO(logger, "On Face %d with confidence %f", face.side, face.confidence);
    }
    static FaceAndConfidence which_side(const cubesat_msgs::msg::AccelSample &sample);
};

} // namespace crashout_cubesat
