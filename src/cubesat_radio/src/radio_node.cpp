#include "cubesat_radio/radio_node.hpp"

#include <chrono>
#include <utility>

namespace cubesat_radio {

RadioNode::RadioNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("radio_node", options) {
  const auto hardware = loadHardwareConfig();
  const auto profile = loadProfile();

  rxPacketPub =
      create_publisher<cubesat_msgs::msg::RadioPacket>("radio/rx_packet", 10);
  txPacketSub = create_subscription<cubesat_msgs::msg::RadioPacket>(
      "radio/tx_packet", 10,
      std::bind(&RadioNode::handleTxPacket, this, std::placeholders::_1));
  sendRadioPacketSrv = create_service<cubesat_msgs::srv::SendRadioPacket>(
      "radio/send_packet",
      std::bind(&RadioNode::handleSendRadioPacket, this, std::placeholders::_1,
                std::placeholders::_2));
  radio = std::make_unique<Sx1262Radio>(hardware);

  if (!radio->open()) {
    RCLCPP_WARN(get_logger(), "Radio hardware open failed; verify SPI device, "
                              "GPIO lines, and SX1262 wiring");
    return;
  }

  if (!radio->configure(profile)) {
    RCLCPP_WARN(get_logger(), "Radio configure failed; verify LoRa profile and "
                              "SX1262 initialization sequence");
    return;
  }

  if (!radio->setReceiveMode()) {
    RCLCPP_WARN(get_logger(),
                "Radio RX mode enable failed after configuration");
    return;
  }

  RCLCPP_INFO(get_logger(),
              "Radio initialized: freq=%u Hz bw=%u Hz sf=%u cr=4/%u tx=%d dBm "
              "spi=%s reset=%d busy=%d dio1=%d",
              profile.frequency_hz, profile.bandwidth_hz,
              profile.spreading_factor, profile.coding_rate,
              profile.tx_power_dbm, hardware.spi_device.c_str(),
              hardware.reset_gpio, hardware.busy_gpio, hardware.dio1_gpio);

  running.store(true);
  rxThread = std::thread(&RadioNode::receiveLoop, this);
}

RadioNode::~RadioNode() {
  running.store(false);
  if (rxThread.joinable()) {
    rxThread.join();
  }
}

RadioProfile RadioNode::loadProfile() {
  RadioProfile profile;
  profile.frequency_hz = static_cast<uint32_t>(
      declare_parameter<int64_t>("frequency_hz", profile.frequency_hz));
  profile.bandwidth_hz = static_cast<uint32_t>(
      declare_parameter<int64_t>("bandwidth_hz", profile.bandwidth_hz));
  profile.spreading_factor = static_cast<uint8_t>(
      declare_parameter<int64_t>("spreading_factor", profile.spreading_factor));
  profile.coding_rate = static_cast<uint8_t>(
      declare_parameter<int64_t>("coding_rate", profile.coding_rate));
  profile.tx_power_dbm = static_cast<int8_t>(
      declare_parameter<int64_t>("tx_power_dbm", profile.tx_power_dbm));
  return profile;
}

RadioHardwareConfig RadioNode::loadHardwareConfig() {
  RadioHardwareConfig hardware;
  hardware.spi_device =
      declare_parameter<std::string>("spi_device", hardware.spi_device);
  hardware.spi_speed_hz = static_cast<uint32_t>(
      declare_parameter<int64_t>("spi_speed_hz", hardware.spi_speed_hz));
  hardware.gpio_chip_name =
      declare_parameter<std::string>("gpio_chip_name", hardware.gpio_chip_name);
  hardware.reset_gpio = static_cast<int>(
      declare_parameter<int64_t>("reset_gpio", hardware.reset_gpio));
  hardware.busy_gpio = static_cast<int>(
      declare_parameter<int64_t>("busy_gpio", hardware.busy_gpio));
  hardware.dio1_gpio = static_cast<int>(
      declare_parameter<int64_t>("dio1_gpio", hardware.dio1_gpio));
  hardware.tx_enable_gpio = static_cast<int>(
      declare_parameter<int64_t>("tx_enable_gpio", hardware.tx_enable_gpio));
  hardware.rx_enable_gpio = static_cast<int>(
      declare_parameter<int64_t>("rx_enable_gpio", hardware.rx_enable_gpio));
  hardware.tx_enable_active_high = declare_parameter<bool>(
      "tx_enable_active_high", hardware.tx_enable_active_high);
  hardware.rx_enable_active_high = declare_parameter<bool>(
      "rx_enable_active_high", hardware.rx_enable_active_high);
  return hardware;
}

void RadioNode::receiveLoop() {
  while (rclcpp::ok() && running.load()) {
    if (txActive.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!radio->waitForInterrupt(std::chrono::milliseconds(200))) {
      continue;
    }

    if (txActive.load()) {
      continue;
    }

    auto packet = radio->receive();
    if (!packet) {
      continue;
    }

    cubesat_msgs::msg::RadioPacket msg;
    msg.stamp = now();
    msg.data = std::move(packet->data);
    msg.rssi = packet->rssi_dbm;
    msg.snr = packet->snr_db;
    rxPacketPub->publish(std::move(msg));
  }
}

bool RadioNode::sendPacket(const std::vector<uint8_t> &data) {
  if (radio == nullptr || data.empty()) {
    return false;
  }

  txActive.store(true);
  const bool success = radio->send(data);
  txActive.store(false);

  if (!success) {
    RCLCPP_WARN(get_logger(), "Radio TX failed for %zu bytes", data.size());
  }
  return success;
}

void RadioNode::handleTxPacket(
    const cubesat_msgs::msg::RadioPacket::SharedPtr msg) {
  if (msg == nullptr) {
    return;
  }

  if (sendPacket(msg->data)) {
    RCLCPP_INFO(get_logger(), "Queued radio TX from topic: %zu bytes",
                msg->data.size());
  }
}

void RadioNode::handleSendRadioPacket(
    const std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Request> &request,
    std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Response> response) {
  response->success = request != nullptr && sendPacket(request->data);
}

} // namespace cubesat_radio
