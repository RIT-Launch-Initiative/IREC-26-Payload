#include "cubesat_captain/flight_expert.hpp"

namespace cubesat_captain {
FlightExpert::FlightExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}

void FlightExpert::enter_state() {
    RCLCPP_INFO(logger, "Starting flight");
    levers.start_flight_timer();
    levers.status.set_takeoff_time();
    levers.set_runcam_power(true);

    auto request = std::make_shared<cubesat_msgs::srv::HoldShut::Request>();
    request->should_hold = true;
    levers.hold_shut_client->async_send_request(request);
}

void FlightExpert::exit_state() {
    auto request = std::make_shared<cubesat_msgs::srv::HoldShut::Request>();
    request->should_hold = false;
    levers.hold_shut_client->async_send_request(request);

    levers.stop_flight_timer();
}

void FlightExpert::handle_flight_timer_expired() {
    RCLCPP_INFO(logger, "Flight Timer expired");
    levers.goto_state(State::Flipping);
}

} // namespace cubesat_captain