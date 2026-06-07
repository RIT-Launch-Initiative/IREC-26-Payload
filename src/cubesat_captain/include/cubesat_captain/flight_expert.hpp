#include "cubesat_captain/expert.hpp"

namespace cubesat_captain {

class FlightExpert : public Expert {
  public:
    FlightExpert(rclcpp::Logger logger, Levers &levers);
    ~FlightExpert() {}

    void handle_flight_timer_expired() override;
    void enter_state() override;
    void exit_state() override;

  private:
    rclcpp::TimerBase::SharedPtr flight_time_timer;
};

} // namespace cubesat_captain