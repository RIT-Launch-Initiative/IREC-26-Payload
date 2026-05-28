#pragma once

#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/srv/request_state_change.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cubesat_captain {

class CaptainNode : public rclcpp::Node {
  public:
    explicit CaptainNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    enum class State {
        Pad = cubesat_msgs::msg::FlightState::STATE_PAD,
        Preboost = cubesat_msgs::msg::FlightState::STATE_PREBOOST,
        Flight = cubesat_msgs::msg::FlightState::STATE_FLIGHT,
        Flipping = cubesat_msgs::msg::FlightState::STATE_FLIPPING,
        Unfolding = cubesat_msgs::msg::FlightState::STATE_UNFOLDING,
        AutoCamera = cubesat_msgs::msg::FlightState::STATE_AUTO_CAMERA,
        ManualControl = cubesat_msgs::msg::FlightState::STATE_MANUAL_CONTROL,
        Emergency = cubesat_msgs::msg::FlightState::STATE_EMERGENCY,
    };

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

  private:
    void change_internal_state(State state);
    void enter_pad();
    void enter_preboost();
    void enter_flight();
    void enter_flipping();
    void enter_unfolding();
    void enter_auto_camera();
    void enter_manual();
    void enter_emergency();

    void handleImu(cubesat_msgs::msg::AccelState &sample);

    State current_state = State::Pad;
    std::string flight_dir;
    double flight_time_s;
    double pad_heartbeat_s;
    double flight_heartbeat_s;
    double landed_heartbeat_s;
    double boost_threshold_mps2;

    rclcpp::Service<cubesat_msgs::srv::RequestStateChange>::SharedPtr request_state_change_service;
    rclcpp::Publisher<cubesat_msgs::msg::FlightState>::SharedPtr state_pub;
};

} // namespace cubesat_captain
