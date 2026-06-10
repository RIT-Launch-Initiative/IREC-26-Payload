#pragma once
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/arm_status.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/gps_sample.hpp"
#include "cubesat_msgs/msg/power_sample.hpp"

namespace cubesat_captain {

struct Parameters {
    double flight_time_s;
    double pad_heartbeat_s;
    double secondary_heartbeat_s;
    double primary_heartbeat_s;
    double boost_threshold_mps2;

    double warn_battery_threshold_v;
    double danger_battery_threshold_v;
};

enum class State {
    Pad = cubesat_msgs::msg::FlightState::STATE_PAD,
    Preboost = cubesat_msgs::msg::FlightState::STATE_PREBOOST,
    Flight = cubesat_msgs::msg::FlightState::STATE_FLIGHT,
    Flipping = cubesat_msgs::msg::FlightState::STATE_FLIPPING,
    Unfolding = cubesat_msgs::msg::FlightState::STATE_UNFOLDING,
    AutoCamera = cubesat_msgs::msg::FlightState::STATE_AUTO_CAMERA,
    ManualControl = cubesat_msgs::msg::FlightState::STATE_MANUAL_CONTROL,
    Emergency = cubesat_msgs::msg::FlightState::STATE_EMERGENCY,
    NumStates = 8,
};

class StatusAccumulator {
  public:
    bool load_from_folder();
    void update_base_accel(const cubesat_msgs::msg::AccelSample &sample);
    void update_arm_status(const cubesat_msgs::msg::ArmStatus &status);
    void update_gps_sample(const cubesat_msgs::msg::GpsSample &sample);
    void update_power_sample(const cubesat_msgs::msg::PowerSample &sample);
    void update_last_image(uint8_t just_taken_image);

    void update_flight_state(const cubesat_msgs::msg::FlightState &);

    void set_takeoff_time();
    void clear_takeoff_time();

    State active_state() const;

    // if the most recent gps packet had fix
    bool has_gps() const;
    // last known gps coordinates if we ever got a fix
    void last_good_gps_position(float *lat, float *lon, float *alt) const;

    Parameters current_parameters;

    cubesat_msgs::msg::FlightState current_state{};
    cubesat_msgs::msg::GpsSample last_gps_sample{};
    cubesat_msgs::msg::GpsSample last_good_gps_sample{};
    cubesat_msgs::msg::ArmStatus last_arm_status{};
    cubesat_msgs::msg::PowerSample last_power_sample{};

    cubesat_msgs::msg::AccelSample last_base_accel{};
    uint8_t last_image_id = 0;
    uint32_t takeoff_time = 0;

  private:
};

} // namespace cubesat_captain