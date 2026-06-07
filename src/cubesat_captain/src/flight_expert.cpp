#include "cubesat_captain/flight_expert.hpp"

namespace cubesat_captain {
FlightExpert::FlightExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}


void FlightExpert::enter_state() {
    levers.start_flight_timer();
    levers.status.set_takeoff_time();
    levers.set_runcam_power(true);
}

void FlightExpert::exit_state() { levers.stop_flight_timer(); }

void FlightExpert::handle_flight_timer_expired() { levers.goto_state(State::Flipping); }

} // namespace cubesat_captain