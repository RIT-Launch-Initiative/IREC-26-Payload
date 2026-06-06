#include "cubesat_captain/captain_node.hpp"
#include "cubesat_captain/flipping_state.hpp"
#include "cubesat_captain/packet_writer.hpp"
#include "cubesat_captain/pad_state.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace cubesat_captain {

CaptainNode::CaptainNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("captain_node", options), status{},
      levers{status,
             create_client<cubesat_msgs::srv::JogMotor>("/stm/jog_motor"),
             create_client<cubesat_msgs::srv::HoldShut>("/stm/hold_shut"),
             create_client<cubesat_msgs::srv::ZeroArm>("/stm/zero_arm"),
             rclcpp_action::create_client<cubesat_msgs::action::ExtendArm>(this, "/stm/flip_servo"),
             rclcpp_action::create_client<cubesat_msgs::action::FlipServoAction>(this, "/stm/flip_servo"),
             [this](State state) { this->change_internal_state(state); }} {
    flight_dir = declare_parameter<std::string>("flight_dir", "~/unconfigured_flight_dir");
    load_startup_parameters();

    RCLCPP_INFO(get_logger(), "Captain Node started: Flight Time: %f", status.current_parameters.flight_time_s);
    RCLCPP_INFO(get_logger(), "Pad HB: %f s, Flight HB %f s, Landed HB %f s", status.current_parameters.pad_heartbeat_s,
                status.current_parameters.flight_heartbeat_s, status.current_parameters.landed_heartbeat_s);

    RCLCPP_INFO(get_logger(), "Battery Low %.2f V, Dangerous %.2f", status.current_parameters.warn_battery_threshold_v,
                status.current_parameters.danger_battery_threshold_v);

    if (!std::filesystem::exists(flight_dir)) {
        RCLCPP_ERROR(get_logger(), "flight_dir doesnt exist. Creating");
        std::filesystem::create_directory(flight_dir);
    }
    if (!std::filesystem::is_directory(flight_dir)) {
        RCLCPP_ERROR(get_logger(), "flight_dir isn't a directory??");
    }

    state_pub = create_publisher<cubesat_msgs::msg::FlightState>("pi/flight_state", 10);
    image_req_pub = create_publisher<cubesat_msgs::msg::ImageRequest>("/watcher/image_request", 10);

    radio_packet_pub = create_publisher<cubesat_msgs::msg::RadioPacket>("/radio/tx_packet", 68);

    imu_sub = create_subscription<cubesat_msgs::msg::AccelSample>(
        "pi/lis3dh", 10, std::bind(&CaptainNode::handle_imu, this, std::placeholders::_1));

    power_sub = create_subscription<cubesat_msgs::msg::PowerSample>(
        "pi/power", 10, std::bind(&CaptainNode::handle_power, this, std::placeholders::_1));

    gnss_sub = create_subscription<cubesat_msgs::msg::GpsSample>(
        "pi/gps", 10, std::bind(&CaptainNode::handle_gnss, this, std::placeholders::_1));

    radio_sub = create_subscription<cubesat_msgs::msg::RadioPacket>(
        "radio/rx_packet", 10, std::bind(&CaptainNode::handle_packet, this, std::placeholders::_1));

    radio_state_sub = create_subscription<cubesat_msgs::msg::RadioState>(
        "radio/state", 10, std::bind(&CaptainNode::handle_radio_state, this, std::placeholders::_1));

    image_metadata_sub = create_subscription<cubesat_msgs::msg::ImageMetadata>(
        "watcher/image_metadata", 10, std::bind(&CaptainNode::handle_image_metadata, this, std::placeholders::_1));

    this->request_state_change_service = create_service<cubesat_msgs::srv::RequestStateChange>(
        "/pi/change_state",
        std::bind(&CaptainNode::requestStateChange, this, std::placeholders::_1, std::placeholders::_2));

    this->request_telemetry_service = create_service<cubesat_msgs::srv::TelemetryRequest>(
        "/pi/request_telemetry",
        std::bind(&CaptainNode::requestTelemetry, this, std::placeholders::_1, std::placeholders::_2));

    this->set_buzzer_client = create_client<cubesat_msgs::srv::SetBuzzer>("/pi/buzzer");

    heartbeat_timer =
        create_wall_timer(std::chrono::milliseconds((int)(1000 * status.current_parameters.pad_heartbeat_s)),
                          [this] { onHeartbeatTimer(); });

    PadExpert *pad_expert = new PadExpert(get_logger(), levers);
    experts[(int)State::Pad] = pad_expert;
    experts[(int)State::Preboost] = new PreboostExpert(get_logger(), levers, pad_expert); // bad and terrible ngl
    experts[(int)State::Flipping] = new FlippingExpert(get_logger(), levers);

    // enter initial state
    State initial_state = State::Pad;

    cubesat_msgs::msg::FlightState msg;
    msg.stamp = now();
    msg.state = (uint8_t)initial_state;
    status.update_flight_state(msg);
    state_pub->publish(msg);

    Expert *expert = expert_for_state(initial_state);
    if (expert != nullptr) {
        expert->enter_state();
    }
}

void CaptainNode::requestStateChange(const std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Request> request,
                                     std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Response> response) {
    State to_state = (State)request->to_state.state;
    if (to_state >= State::NumStates) {
        response->success = true;
        response->reason = "invalid state";
        return;
    }
    response->success = true;
    change_internal_state(to_state);
}

void CaptainNode::load_startup_parameters() {
    status.current_parameters.flight_time_s = declare_parameter<double>("flight_time_s", 100);
    status.current_parameters.pad_heartbeat_s = declare_parameter<double>("pad_heartbeat_s", 0.05);
    status.current_parameters.flight_heartbeat_s = declare_parameter<double>("flight_heartbeat_s", 0.2);
    status.current_parameters.landed_heartbeat_s = declare_parameter<double>("landed_heartbeat_s", 0.2);
    status.current_parameters.boost_threshold_mps2 = declare_parameter<double>("boost_threshold_mps2", 7);

    status.current_parameters.warn_battery_threshold_v = declare_parameter<double>("warn_battery_threshold_v", 10.75);
    status.current_parameters.danger_battery_threshold_v =
        declare_parameter<double>("danger_battery_threshold_v", 10.5);
}

void CaptainNode::change_internal_state(State state) {
    State old_state = status.active_state();
    Expert *old_expert = expert_for_state(old_state);
    if (old_expert != nullptr) {
        old_expert->exit_state();
    }

    cubesat_msgs::msg::FlightState msg;
    msg.stamp = now();
    msg.state = (uint8_t)state;
    status.update_flight_state(msg);
    state_pub->publish(msg);

    Expert *expert = expert_for_state(state);
    if (expert != nullptr) {
        expert->enter_state();
    }
}

void CaptainNode::flag_for_new_flight_dir() { std::ofstream(flight_dir + "/new_dir_please.flag").close(); }

void CaptainNode::restart_system() {
    RCLCPP_WARN(get_logger(), "Restarting self (except not actually bc not implemented)");
}

void CaptainNode::handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample) {
    status.update_base_accel(*sample);

    Expert *expert = expert_for_state(status.active_state());
    if (expert != nullptr) {
        expert->handle_base_accel(*sample);
    }
}

void CaptainNode::handle_power(const cubesat_msgs::msg::PowerSample::SharedPtr sample) {
    status.update_power_sample(*sample);

    bool battery_low = sample->bus_voltage_v < status.current_parameters.warn_battery_threshold_v;
    bool battery_dangerous = sample->bus_voltage_v < status.current_parameters.danger_battery_threshold_v;

    Expert *expert = expert_for_state(status.active_state());

    if (expert != nullptr) {
        expert->handle_power_sample(*sample);
    }

    // send to buzzer (we don't actually care if it gets there so don't spin for result)
    auto request = std::make_shared<cubesat_msgs::srv::SetBuzzer::Request>();
    if (battery_low && !was_battery_low) {
        RCLCPP_WARN(get_logger(), "BATTERY LOW");
        request->repeat_count = 100;
        request->beep_code = cubesat_msgs::srv::SetBuzzer::Request::BEEP_CODE_3_EQUAL;
        set_buzzer_client->async_send_request(request);
    }
    if (battery_dangerous && !was_battery_dangerous) {
        RCLCPP_WARN(get_logger(), "BATTERY DANGEROUSLY LOW");
        request->repeat_count = 10;
        request->beep_code = cubesat_msgs::srv::SetBuzzer::Request::BEEP_CODE_SMALL;
        set_buzzer_client->async_send_request(request);
    }
    was_battery_low = battery_low;
    was_battery_dangerous = battery_dangerous;
}

void CaptainNode::handle_gnss(const cubesat_msgs::msg::GpsSample::SharedPtr sample) {
    status.update_gps_sample(*sample);
}

Expert *CaptainNode::expert_for_state(State state) {
    if (state > State::NumStates) {
        return nullptr;
    }
    return experts[(int)state];
}

void CaptainNode::emit_telemetry(cubesat_msgs::msg::TelemetryType telem_type) {
    cubesat_msgs::msg::RadioPacket pkt;
    pkt.data.resize(255);
    int size = packet_for_telemetry(status, telem_type, pkt.data.data());

    if (size < 0) {
        RCLCPP_WARN(get_logger(), "Invalid request for telemetry. not sending");
        return;
    }
    pkt.data.resize(size);

    radio_packet_pub->publish(pkt);
}

void CaptainNode::onHeartbeatTimer() {
    cubesat_msgs::msg::TelemetryType typ;
    typ.telem_id = cubesat_msgs::msg::TelemetryType::FLIGHT_HEARTBEAT;
    emit_telemetry(typ);
}

void CaptainNode::requestTelemetry(const std::shared_ptr<cubesat_msgs::srv::TelemetryRequest::Request> request,
                                   std::shared_ptr<cubesat_msgs::srv::TelemetryRequest::Response> response) {
    RCLCPP_INFO(get_logger(), "Telemetry Requested: type %d", request->telemetry.telem_id);
    emit_telemetry(request->telemetry);
    response->success = true;
}

void CaptainNode::handle_radio_state(const cubesat_msgs::msg::RadioState::SharedPtr state) {
    RCLCPP_INFO(get_logger(), "Radio state changed: queue length %d", state->queue_depth);
}

void CaptainNode::handle_image_metadata(const cubesat_msgs::msg::ImageMetadata::SharedPtr meta) {
    RCLCPP_INFO(get_logger(), "Image Metadata delivered for image %d", (int)meta->image_id);
    status.update_last_image(meta->image_id);
    emit_image_metadata(*meta);
}

} // namespace cubesat_captain
