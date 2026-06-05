#include "cubesat_captain/captain_node.hpp"

#include "cubesat_comms/packets_g2p.h"
namespace cubesat_captain {

void CaptainNode::handle_packet(const cubesat_msgs::msg::RadioPacket::SharedPtr packet) {
    G2PLinkHeader header;
    UnpackResult res = unpack_g2p_link_header(packet->data.data(), packet->data.size(), &header);
    if (res != UnpackResult_AllGood) {
        RCLCPP_WARN(get_logger(), "Bad packet header: %d", (int)res);
        return;
    }
    if (header.packet_type != G2PPacketType_Command) {
        RCLCPP_INFO(get_logger(), "Ignoring packet of type %d bc not command", header.packet_type);
        return;
    }

    CommandAndData cmd_and_data;
    cmd_and_data.command = Command_Callsign;
    UnpackResult body_res = unpack_command_and_data(packet->data.data() + 1, packet->data.size()-1, &cmd_and_data);
    if (body_res != UnpackResult_AllGood) {
        RCLCPP_WARN(get_logger(), "Bad g2p packet body: %d (cmd = %d)", (int)body_res, (int)cmd_and_data.command);
        return;
    }

    switch (cmd_and_data.command) {
    case Command_ForceManual:
        change_internal_state(State::ManualControl);
        break;
    case Command_ForceFlight:
        change_internal_state(State::Flight);
        break;
    case Command_ExpectFlight:
        change_internal_state(State::Preboost);
        break;
    case Command_UnexpectFlight:
        change_internal_state(State::Pad);
        break;
    case Command_TelemetryRequest: {
        cubesat_msgs::msg::TelemetryType telem_type;
        telem_type.telem_id = cmd_and_data.telem_request.telem_type;
        RCLCPP_INFO(get_logger(), "Radio requested telemetry of ype %d", telem_type.telem_id);
        emit_telemetry(telem_type);
    } break;
    case Command_StartVideo:
        // TODO
        break;
    case Command_StopVideo:
        // TODO
        break;
    case Command_TakePicture:{
        cubesat_msgs::msg::ImageRequest req;
        req.left = cmd_and_data.take_picture.left;
        req.right = cmd_and_data.take_picture.right;
        req.top = cmd_and_data.take_picture.top;
        req.bottom = cmd_and_data.take_picture.bottom;
        req.output_width = cmd_and_data.take_picture.output_width;
        req.quality = cmd_and_data.take_picture.quality;
        
        status.last_good_gps_position(&req.latitude, &req.longitude, &req.altitude);
        
        req.arm_shoulder_yaw = status.last_arm_status.shoulder_yaw_deg;
        req.arm_shoulder_pitch = status.last_arm_status.shoulder_pitch_deg;
        req.arm_elbow_pitch = status.last_arm_status.elbow_angle_deg;
        req.arm_wrist_pitch = status.last_arm_status.wrist_angle_deg;
        image_req_pub->publish(req);
    }
    break;
    default:
        RCLCPP_WARN(get_logger(), "Unimplemented g2p packet handler: %d", (int)cmd_and_data.command);
        break;
    }
}
} // namespace cubesat_captain