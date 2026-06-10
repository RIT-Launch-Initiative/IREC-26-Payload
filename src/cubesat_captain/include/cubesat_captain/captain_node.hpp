#pragma once

#include "cubesat_captain/status_accumulator.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"
#include "cubesat_msgs/msg/flight_state.hpp"
#include "cubesat_msgs/msg/power_sample.hpp"

#include "cubesat_msgs/msg/image_metadata.hpp"
#include "cubesat_msgs/msg/image_request.hpp"
#include "cubesat_msgs/msg/radio_packet.hpp"
#include "cubesat_msgs/msg/radio_state.hpp"
#include "cubesat_msgs/msg/telemetry_type.hpp"

#include "cubesat_msgs/srv/request_state_change.hpp"
#include "cubesat_msgs/srv/send_radio_packet.hpp"
#include "cubesat_msgs/srv/set_buzzer.hpp"
#include "cubesat_msgs/srv/telemetry_request.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>

#include "cubesat_captain/expert.hpp"

#include <gpiod.h>

namespace cubesat_captain {

class CaptainNode : public rclcpp::Node {
  public:
    explicit CaptainNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // request state service/handler
    // - radio asks for flight
    // - radio says return to pad
    // - radio asks for manual

    /**
     * Creates flag file that tells launch file to start you in a new directory
     */
    void flag_for_new_flight_dir();
    // hit that systemctl restart to rerun launch script
    void restart_system();

    void requestStateChange(const std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Request> request,
                            std::shared_ptr<cubesat_msgs::srv::RequestStateChange::Response> response);

  private:
    void requestTelemetry(const std::shared_ptr<cubesat_msgs::srv::TelemetryRequest::Request> request,
                          std::shared_ptr<cubesat_msgs::srv::TelemetryRequest::Response> response);

    void load_startup_parameters();
    void change_internal_state(State state);

    Expert *expert_for_state(State state);

    void emit_telemetry(cubesat_msgs::msg::TelemetryType telem_type);
    void emit_imagedata(uint8_t image_id, const std::vector<uint16_t> &blocks);
    void emit_image_metadata(const cubesat_msgs::msg::ImageMetadata &metadata);

    bool loadImageMetadata(std::string path, cubesat_msgs::msg::ImageMetadata &metadata);

    void ask_for_image(uint16_t left, uint16_t right, uint16_t top, uint16_t bottom, uint16_t output_width,
                       uint8_t quality);

    void handle_imu(const cubesat_msgs::msg::AccelSample::SharedPtr sample);
    void handle_power(const cubesat_msgs::msg::PowerSample::SharedPtr sample);
    void handle_gnss(const cubesat_msgs::msg::GpsSample::SharedPtr sample);
    void handle_packet(const cubesat_msgs::msg::RadioPacket::SharedPtr packet);
    void handle_radio_state(const cubesat_msgs::msg::RadioState::SharedPtr state);
    void handle_image_metadata(const cubesat_msgs::msg::ImageMetadata::SharedPtr state);

    void onPrimaryHeartbeatTimer();
    void onSecondaryHeartbeatTimer();

    void onCallsignTimer(); // TODO transmit callsign every once and a while

    StatusAccumulator status;
    Levers levers;
    Expert *experts[(int)State::NumStates] = {nullptr};

    std::string flight_dir;
    std::string restart_command;
    int runcam_pin = 1;
    std::string gpio_chip_name;
    bool was_battery_dangerous{false};
    bool was_battery_low{false};

    gpiod_chip *gpio_chip{nullptr};
    gpiod_line *camera_gpio{nullptr};
    bool openCameraLine();
    bool setCamera(bool on);

    rclcpp::TimerBase::SharedPtr primary_heartbeat_timer;
    rclcpp::TimerBase::SharedPtr secondary_heartbeat_timer;
    rclcpp::TimerBase::SharedPtr flight_timer;

    cubesat_msgs::msg::TelemetryType primary_heartbeat_type;

    rclcpp::Subscription<cubesat_msgs::msg::AccelSample>::SharedPtr imu_sub;
    rclcpp::Subscription<cubesat_msgs::msg::PowerSample>::SharedPtr power_sub;
    rclcpp::Subscription<cubesat_msgs::msg::GpsSample>::SharedPtr gnss_sub;
    rclcpp::Subscription<cubesat_msgs::msg::RadioPacket>::SharedPtr radio_sub;
    rclcpp::Subscription<cubesat_msgs::msg::ArmStatus>::SharedPtr arm_status_sub;

    rclcpp::Subscription<cubesat_msgs::msg::RadioState>::SharedPtr radio_state_sub;
    rclcpp::Subscription<cubesat_msgs::msg::ImageMetadata>::SharedPtr image_metadata_sub;

    rclcpp::Publisher<cubesat_msgs::msg::RadioPacket>::SharedPtr radio_packet_pub;
    rclcpp::Publisher<cubesat_msgs::msg::ImageRequest>::SharedPtr image_req_pub;

    rclcpp::Publisher<cubesat_msgs::msg::FlightState>::SharedPtr state_pub;

    // services that we are the server for
    rclcpp::Service<cubesat_msgs::srv::RequestStateChange>::SharedPtr request_state_change_service;
    rclcpp::Service<cubesat_msgs::srv::TelemetryRequest>::SharedPtr request_telemetry_service;

    // services that we are the client for
    rclcpp::Client<cubesat_msgs::srv::SetBuzzer>::SharedPtr set_buzzer_client;
};

} // namespace cubesat_captain
