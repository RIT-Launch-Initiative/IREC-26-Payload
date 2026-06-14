#include "cubesat_captain/captain_node.hpp"
#include "cubesat_comms/packets_g2p.h"
#include "cubesat_comms/packets_p2g.h"
#include "rclcpp/serialization.hpp"
#include <fstream>
namespace cubesat_captain {

cubesat_msgs::action::ExtendArm::Goal packetToGoal(const ArmTarget &target) {
    cubesat_msgs::action::ExtendArm::Goal goal;

    goal.shoulder_yaw = target.shoulder_yaw;
    goal.shoulder_pitch = target.shoulder_pitch;
    goal.elbow_pitch = target.elbow_pitch;
    goal.wrist_pitch = target.wrist_pitch;

    return goal;
}

void CaptainNode::handle_packet(const cubesat_msgs::msg::RadioPacket::SharedPtr packet) {
    Expert *expert = expert_for_state(status.active_state());

    G2PLinkHeader header;
    UnpackResult res = unpack_g2p_link_header(packet->data.data(), packet->data.size(), &header);
    if (res != UnpackResult_AllGood) {
        RCLCPP_WARN(get_logger(), "Bad packet header: %d", (int)res);
        return;
    }
    if (header.packet_type == G2PPacketType_ImageControl) {
        ImageBlockRequest req;
        UnpackResult body_res = unpack_image_block_request(packet->data.data() + 1, packet->data.size() - 1, &req);
        if (body_res != UnpackResult_AllGood) {
            RCLCPP_WARN(get_logger(), "Couldnt unpack image block request for reason %d", (int)body_res);
            return;
        }
        std::vector<uint16_t> blocks{req.block_ids, req.block_ids + req.num};
        emit_imagedata(req.image_id, blocks);
        return;
    }
    if (header.packet_type != G2PPacketType_Command) {
        RCLCPP_INFO(get_logger(), "Ignoring packet of type %d bc not command", header.packet_type);
        return;
    }

    CommandAndData cmd_and_data;
    cmd_and_data.command = Command_Callsign;
    UnpackResult body_res = unpack_command_and_data(packet->data.data() + 1, packet->data.size() - 1, &cmd_and_data);
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
    case Command_BackToPad:
        change_internal_state(State::Pad);
        break;
    case Command_TelemetryRequest: {
        cubesat_msgs::msg::TelemetryType telem_type;
        telem_type.telem_id = cmd_and_data.telem_request.telem_type;
        RCLCPP_INFO(get_logger(), "Radio requested telemetry of ype %d", telem_type.telem_id);
        emit_telemetry(telem_type);
    } break;
    case Command_SendArmTarget: {
        if (expert != nullptr) {
            auto goal = packetToGoal(cmd_and_data.send_arm_to_target);
            expert->send_arm_to_target(goal);
        }
    } break;
    case Command_SendArmTargetAndComeBack: {
        if (expert != nullptr) {
            auto goal = packetToGoal(cmd_and_data.send_arm_to_target_and_come_back);
            expert->send_arm_to_target_and_come_back(goal);
        }
    } break;
    case Command_SendArmTargetForPhotoAndComeBack: {
        if (expert != nullptr) {
            auto goal = packetToGoal(cmd_and_data.send_arm_to_target_for_photo_and_come_back);
            expert->send_arm_to_target_and_come_back_with_photo(goal);
        }
    } break;
    case Command_JogMotor: {
        auto request = std::make_shared<cubesat_msgs::srv::JogMotor::Request>();
        request->motor = cmd_and_data.motor_jog.motor_id;
        request->milliseconds = cmd_and_data.motor_jog.duration_ticks * 10;
        request->voltage = (float)cmd_and_data.motor_jog.millivolts / 1000.0F;

        levers.jog_motor_client->async_send_request(request);
    } break;
    case Command_MoveServo: {
        Expert *expert = expert_for_state(status.active_state());
        if (expert != nullptr) {
            cubesat_msgs::action::FlipServoAction::Goal goal;
            goal.openness = cmd_and_data.servo_motion.openness;
            goal.open_travel_duration = cmd_and_data.servo_motion.open_travel_time;
            goal.open_duration = cmd_and_data.servo_motion.open_time;
            goal.closedness = cmd_and_data.servo_motion.closeness;
            goal.close_travel_duration = cmd_and_data.servo_motion.close_travel_time;
            expert->execute_servo_motion(goal);
        }

    } break;
    case Command_SetShoulder: {
        auto request = std::make_shared<cubesat_msgs::srv::ZeroArm::Request>();
        request->shoulder_yaw = cmd_and_data.set_shoulder_position.shoulder_yaw;
        request->shoulder_pitch = cmd_and_data.set_shoulder_position.shoulder_pitch;
        request->elbow_angle = cmd_and_data.set_shoulder_position.elbow_pitch;
        request->wrist_angle = cmd_and_data.set_shoulder_position.wrist_pitch;

        levers.zero_arm_client->async_send_request(request);
    } break;
    case Command_StartVideo:
        levers.set_runcam_power(true);
        break;
    case Command_StopVideo:
        levers.set_runcam_power(false);
        break;
    case Command_TakePicture: {
        ask_for_image(cmd_and_data.take_picture.left, cmd_and_data.take_picture.right, cmd_and_data.take_picture.top,
                      cmd_and_data.take_picture.bottom, cmd_and_data.take_picture.output_width,
                      cmd_and_data.take_picture.quality);
    } break;

    case Command_ImageMetadata: {
        uint8_t image_id = cmd_and_data.metadata_ask_image_id;
        // deserialize from filesystem
        std::string path = flight_dir + "/images/" + std::to_string(image_id) + "/metadata.bin";
        cubesat_msgs::msg::ImageMetadata metadata;
        if (!loadImageMetadata(path, metadata)) {
            RCLCPP_WARN(get_logger(), "Dropping metadata request for image %d", image_id);
            break;
        }

        emit_image_metadata(metadata);
    } break;
    default:
        RCLCPP_WARN(get_logger(), "Unimplemented g2p packet handler: %d", (int)cmd_and_data.command);
        break;
    }
}

int load_block_data(std::string path, uint8_t *buf) {
    try {
        // read file
        std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
        file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        if (size != 128) {
            return -1;
        }
        file.read(reinterpret_cast<char *>(buf), size);
        file.close();
    } catch (std::exception &e) {
        return -1;
    }
    return 0;
}

void CaptainNode::emit_imagedata(uint8_t image_id, const std::vector<uint16_t> &blocks) {
    cubesat_msgs::msg::RadioPacket pkt;
    pkt.data.resize(129);
    P2GLinkHeader header{P2GPacketType_ImageResponse, 0};
    pack_p2g_link_header(&header, pkt.data.data());

    for (uint16_t block_id : blocks) {
        std::string path =
            flight_dir + "/images/" + std::to_string(image_id) + "/packets/" + std::to_string(block_id) + ".bin";
        int res = load_block_data(path, &pkt.data.at(1));
        if (res < 0) {
            RCLCPP_WARN(get_logger(), "Failed to load image id %d block id %d to send", image_id, block_id);
            continue;
        }
        radio_packet_pub->publish(pkt);
    }
}

bool CaptainNode::loadImageMetadata(std::string path, cubesat_msgs::msg::ImageMetadata &metadata) {
    try {
        // read file
        std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
        file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<uint8_t> buffer(size);
        file.read(reinterpret_cast<char *>(buffer.data()), size);
        file.close();

        rclcpp::SerializedMessage serialized_msg;
        auto &rcl_msg = serialized_msg.get_rcl_serialized_message();

        // deserialize
        rmw_serialized_message_resize(&rcl_msg, size);
        std::memcpy(rcl_msg.buffer, buffer.data(), size);
        rcl_msg.buffer_length = size; // Explicitly set length to avoid invalid state errors

        rclcpp::Serialization<cubesat_msgs::msg::ImageMetadata> serializer;
        serializer.deserialize_message(&serialized_msg, &metadata);
        return true;
    } catch (std::exception &e) {
        RCLCPP_WARN(get_logger(), "Failed to load image metadata profile from file %s: %s", path.c_str(), e.what());
        return false;
    }
}

void CaptainNode::emit_image_metadata(const cubesat_msgs::msg::ImageMetadata &metadata) {
    // dump it into command
    ImageMetadata meta_msg;
    meta_msg.image_id = metadata.image_id;
    meta_msg.timestamp = metadata.timestamp;
    meta_msg.num_blocks = metadata.num_blocks;

    meta_msg.location.shoulder_yaw = metadata.request.arm_shoulder_yaw;
    meta_msg.location.shoulder_pitch = metadata.request.arm_shoulder_pitch;
    meta_msg.location.elbow_pitch = metadata.request.arm_elbow_pitch;
    meta_msg.location.wrist_pitch = metadata.request.arm_wrist_pitch;

    meta_msg.transform.left = metadata.request.left;
    meta_msg.transform.right = metadata.request.right;
    meta_msg.transform.top = metadata.request.top;
    meta_msg.transform.bottom = metadata.request.bottom;
    meta_msg.transform.output_width = metadata.request.output_width;
    meta_msg.transform.quality = metadata.request.quality;

    meta_msg.latitude = metadata.request.latitude;
    meta_msg.longitude = metadata.request.longitude;

    // packetize
    cubesat_msgs::msg::RadioPacket pkt;
    pkt.data.resize(255);
    P2GLinkHeader header{P2GPacketType_CommandResponse, 0};
    pack_p2g_link_header(&header, pkt.data.data());

    CommandResponse cmd;
    cmd.cmd = Command_ImageMetadata;
    cmd.image_metadata = meta_msg;

    size_t size = pack_command_response(&cmd, &pkt.data.at(1));
    pkt.data.resize(size + 1);
    radio_packet_pub->publish(pkt);
}

} // namespace cubesat_captain