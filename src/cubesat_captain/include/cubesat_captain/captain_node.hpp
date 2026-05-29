#pragma once

#include "cubesat_captain/status_accumulator.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/telemetry_type.hpp"
#include "cubesat_msgs/srv/request_state_change.hpp"
#include "cubesat_msgs/srv/telemetry_request.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

#include "cubesat_captain/expert.hpp"

namespace cubesat_captain {

class CaptainNode : public rclcpp::Node {
  public:
    explicit CaptainNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // request state service/handler
    // - radio asks for flight
    // - radio says return to pad
    // - radio asks for manual

    /**
     * Creates flag file that tells launch file to start you as
     */
    void flag_for_new_flight_dir();
    // hit that systemctl restart to rerun launch script
    void restart_system();

    void requestStateChange(const std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Request> request,
                            std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Response> response);

  private:
    void load_startup_parameters();
    void change_internal_state(State state);

    Expert *expert_for_state(State state);

    void emit_telemetry(cubesat_msgs::msg::TelemetryType telem_type);

    void handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample);

    StatusAccumulator status;
    Levers levers;
    Expert *experts[(int)State::NumStates] = {nullptr};

    std::string flight_dir;

    rclcpp::Subscription<cubesat_msgs::msg::AccelSample>::SharedPtr imu_sub;
    rclcpp::Publisher<cubesat_msgs::msg::FlightState>::SharedPtr state_pub;

    rclcpp::Service<cubesat_msgs::srv::RequestStateChange>::SharedPtr request_state_change_service;
    rclcpp::Service<cubesat_msgs::srv::TelemetryRequest>::SharedPtr request_telemetry;
};

} // namespace cubesat_captain
