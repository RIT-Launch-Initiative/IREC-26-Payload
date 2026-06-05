#include "cubesat_captain/packet_writer.hpp"
#include "cubesat_comms/packets_p2g.h"

namespace cubesat_captain {

FlightPhase flight_phase(const StatusAccumulator &status) {
    switch (status.active_state()) {
    case State::Pad:
        return FlightPhase_Pad;
    case State::Preboost:
        return FlightPhase_ExpectingLaunch;
    case State::Flight:
        return FlightPhase_Flight;
    case State::Flipping:
        return FlightPhase_LandedFlipping;
    case State::Unfolding:
        return FlightPhase_Unfolding;
    case State::AutoCamera:
        return FlightPhase_AutoCamera;
    case State::ManualControl:
        return FlightPhase_ManualControl;
    case State::Emergency:
        return FlightPhase_Emergency;
    default:
        return FlightPhase_Starting;
    };
}
FlightState flight_state(const StatusAccumulator &status) {
    enum FlightPhase phase = flight_phase(status);
    uint16_t status_bits = 0;
    status_bits |= status.has_gps() << StatusBit_GPSHasFix;
    return {phase, status_bits};
}

void packet_for_flight_heartbeat(const StatusAccumulator &status, FlightHeartbeatStats &telem) {
    telem.state = flight_state(status);

    float altitude_m = 0;
    status.last_good_gps_position(&telem.latitude, &telem.longitude, &altitude_m);
    if (altitude_m < 0) {
        altitude_m = 0;
    }
    telem.altitude = (uint16_t)altitude_m;

    telem.s_since_boost = 0;

    if (status.last_power_sample.bus_voltage_v < -32768) {
        telem.battery_mV = -32768;
    } else if (status.last_power_sample.bus_voltage_v > 32767) {
        telem.battery_mV = 32767;
    } else {
        telem.battery_mV = (uint16_t)(status.last_power_sample.bus_voltage_v * 1000);
    }
    telem.radio_temp = 55; // TODO real
}
void packet_for_landed_heartbeat(const StatusAccumulator &status, LandedHeartbeatStats &telem) {
    
}
void packet_for_actuators(const StatusAccumulator &status, Telemetry &telem) {}
void packet_for_gnss(const StatusAccumulator &status, Telemetry &telem) {}
void packet_for_system_stats(const StatusAccumulator &status, Telemetry &telem) {}
void packet_for_orientations(const StatusAccumulator &status, Telemetry &telem) {}
void packet_for_temps(const StatusAccumulator &status, Telemetry &telem) {}
void packet_for_power(const StatusAccumulator &status, Telemetry &telem) {}

int packet_for_telemetry(const StatusAccumulator &status, cubesat_msgs::msg::TelemetryType telem_type, uint8_t *span) {
    P2GLinkHeader header = P2GLinkHeader{P2GPacketType_CommandResponse, 1};
    CommandResponse resp;

    resp.cmd = Command_TelemetryRequest;

    using TT = cubesat_msgs::msg::TelemetryType;
    switch (telem_type.telem_id) {
    case TT::FLIGHT_HEARTBEAT:
        resp.telemetry.telem_type = TelemetryType_FlightHeartbeat;
        packet_for_flight_heartbeat(status, resp.telemetry.flight_heartbeat_stats);
        break;
    case TT::LANDED_HEARTBEAT:
        resp.telemetry.telem_type = TelemetryType_LandedHeartbeat;
        packet_for_landed_heartbeat(status, resp.telemetry.landed_heartbeat_stats);
        break;
    case TT::ACTUATORS:
        resp.telemetry.telem_type = TelemetryType_Actuators;
        packet_for_actuators(status, resp.telemetry);
        break;
    case TT::GNSS:
        resp.telemetry.telem_type = TelemetryType_GNSS;
        packet_for_gnss(status, resp.telemetry);
        break;
    case TT::SYSTEM:
        resp.telemetry.telem_type = TelemetryType_System;
        packet_for_system_stats(status, resp.telemetry);
        break;
    case TT::ORIENTATIONS:
        resp.telemetry.telem_type = TelemetryType_Orientations;
        packet_for_orientations(status, resp.telemetry);
        break;
    case TT::TEMPS:
        resp.telemetry.telem_type = TelemetryType_Temps;
        packet_for_temps(status, resp.telemetry);
        break;
    case TT::POWER:
        resp.telemetry.telem_type = TelemetryType_Power;
        packet_for_power(status, resp.telemetry);
        break;
    default: 
        return -1;
    };

    int actual_size = pack_p2g_link_header(&header, span);
    actual_size += pack_command_response(&resp, span + actual_size);
    return actual_size;
}
} // namespace cubesat_captain
