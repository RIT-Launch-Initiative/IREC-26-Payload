#include "status_accumulator.hpp"
#include "cubesat_msgs/msg/radio_packet.hpp"
#include "cubesat_msgs/msg/telemetry_type.hpp"
namespace cubesat_captain{

 int packet_for_telemetry(const StatusAccumulator &, cubesat_msgs::msg::TelemetryType, uint8_t *span);
}