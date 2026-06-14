#include "cubesat_captain/packet_writer.hpp"
#include "cubesat_captain/common.hpp"
#include "cubesat_comms/packets_p2g.h"
#include <rclcpp/rclcpp.hpp>

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
    status_bits |= status.last_arm_status.motor_en << StatusBit_ArmMoving;
    status_bits |= status.runcam_on << StatusBit_RuncamOn;
    status_bits |= status.last_arm_status.flip_servo_en << StatusBit_ServoMoving;
    status_bits |= status.last_arm_status.cant_trust_imu_link << StatusBit_CantTrustLink2Imu;
    status_bits |= status.last_arm_status.booted << StatusBit_StmBooted;
    return {phase, status_bits};
}

int16_t mv_from_volts(float volts) {
    float battery_mv = volts * 1000;

    if (battery_mv < -32768.0f) {
        return -32768;
    } else if (battery_mv > 32767.0f) {
        return 32767;
    } else {
        return (int16_t)battery_mv;
    }
}

void packet_for_flight_heartbeat(const StatusAccumulator &status, FlightHeartbeatStats &telem) {
    telem.state = flight_state(status);

    float altitude_m = 0;
    status.last_good_gps_position(&telem.latitude, &telem.longitude, &altitude_m);
    if (altitude_m < 0) {
        altitude_m = 0;
    }
    telem.altitude = (uint16_t)altitude_m;

    if (status.takeoff_time == 0) {
        telem.s_since_boost = 0;
    } else {
        rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
        rclcpp::Time now = system_clock.now();
        uint32_t elapsed = (uint32_t)now.seconds() - status.takeoff_time;
        if (elapsed > 65535) {
            elapsed = 65535;
        }
        telem.s_since_boost = (uint16_t)elapsed;
    }

    telem.battery_mV = mv_from_volts(status.last_power_sample.bus_voltage_v);
    telem.radio_temp = 55; // TODO real
}

void packet_for_landed_heartbeat(const StatusAccumulator &status, LandedHeartbeatStats &telem) {
    telem.state = flight_state(status);
    telem.next_image_id = status.last_image_id;
    telem.next_exec_id = 0;
    telem.arm_position.shoulder_yaw = (int8_t)status.last_arm_status.shoulder_yaw_deg;
    telem.arm_position.shoulder_pitch = (int8_t)status.last_arm_status.shoulder_pitch_deg;
    telem.arm_position.elbow_pitch = (int8_t)status.last_arm_status.elbow_angle_deg;
    telem.arm_position.wrist_pitch = (int8_t)status.last_arm_status.wrist_angle_deg;

    telem.motor_temp = 20;
    telem.radio_temp = 20;
    telem.battery_mV = mv_from_volts(status.last_power_sample.bus_voltage_v);
}
void packet_for_actuators([[maybe_unused]] const StatusAccumulator &status, [[maybe_unused]] Telemetry &telem) {}
void packet_for_gnss([[maybe_unused]] const StatusAccumulator &status, [[maybe_unused]] Telemetry &telem) {}
void packet_for_system_stats([[maybe_unused]] const StatusAccumulator &status, [[maybe_unused]] Telemetry &telem) {}
void packet_for_orientations(const StatusAccumulator &status, IMUs &telem) {
    auto saturate16 = [](float normed) -> int16_t {
        if (normed * 32767 > 32767) {
            return 32767;
        } else if (normed * 32767 < -32767) {
            return -32767;
        }
        return (int16_t)(normed * 32767);
    };
    auto base_accel_normed = normalize_accel(status.last_base_accel);

    telem.base.x = saturate16(base_accel_normed.ax);
    telem.base.y = saturate16(base_accel_normed.ay);
    telem.base.z = saturate16(base_accel_normed.az);

    telem.link2.x = saturate16(status.last_arm_status.last_link2_accel.ax / 32767);
    telem.link2.y = saturate16(status.last_arm_status.last_link2_accel.ay / 32767);
    telem.link2.z = saturate16(status.last_arm_status.last_link2_accel.az / 32767);
}
void packet_for_temps([[maybe_unused]] const StatusAccumulator &status, [[maybe_unused]] Telemetry &telem) {}
void packet_for_power([[maybe_unused]] const StatusAccumulator &status, [[maybe_unused]] Telemetry &telem) {}

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
        packet_for_orientations(status, resp.telemetry.orientations);
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
