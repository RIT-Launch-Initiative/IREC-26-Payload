#include "cubesat_radio/radio_node.hpp"

namespace cubesat_radio {

RadioNode::RadioNode(const rclcpp::NodeOptions& options) : rclcpp::Node("radio_node", options) {
    const auto hardware = loadHardwareConfig();
    const auto profile = loadProfile();

    radio_ = std::make_unique<Sx1262Radio>(hardware);

    if (!radio_->open()) {
        RCLCPP_WARN(get_logger(), "Radio hardware open failed; verify SPI device, GPIO lines, and SX1262 wiring");
        return;
    }

    if (!radio_->configure(profile)) {
        RCLCPP_WARN(get_logger(), "Radio configure failed; verify LoRa profile and SX1262 initialization sequence");
        return;
    }

    if (!radio_->setReceiveMode()) {
        RCLCPP_WARN(get_logger(), "Radio RX mode enable failed after configuration");
        return;
    }

    RCLCPP_INFO(get_logger(),
                "Radio initialized: freq=%u Hz bw=%u Hz sf=%u cr=4/%u tx=%d dBm spi=%s reset=%d busy=%d dio1=%d",
                profile.frequency_hz, profile.bandwidth_hz, profile.spreading_factor, profile.coding_rate,
                profile.tx_power_dbm, hardware.spi_device.c_str(), hardware.reset_gpio, hardware.busy_gpio,
                hardware.dio1_gpio);
}

RadioProfile RadioNode::loadProfile() {
    RadioProfile profile;
    profile.frequency_hz = static_cast<uint32_t>(declare_parameter<int64_t>("frequency_hz", profile.frequency_hz));
    profile.bandwidth_hz = static_cast<uint32_t>(declare_parameter<int64_t>("bandwidth_hz", profile.bandwidth_hz));
    profile.spreading_factor =
        static_cast<uint8_t>(declare_parameter<int64_t>("spreading_factor", profile.spreading_factor));
    profile.coding_rate = static_cast<uint8_t>(declare_parameter<int64_t>("coding_rate", profile.coding_rate));
    profile.tx_power_dbm = static_cast<int8_t>(declare_parameter<int64_t>("tx_power_dbm", profile.tx_power_dbm));
    return profile;
}

RadioHardwareConfig RadioNode::loadHardwareConfig() {
    RadioHardwareConfig hardware;
    hardware.spi_device = declare_parameter<std::string>("spi_device", hardware.spi_device);
    hardware.spi_speed_hz = static_cast<uint32_t>(declare_parameter<int64_t>("spi_speed_hz", hardware.spi_speed_hz));
    hardware.gpio_chip_name = declare_parameter<std::string>("gpio_chip_name", hardware.gpio_chip_name);
    hardware.reset_gpio = static_cast<int>(declare_parameter<int64_t>("reset_gpio", hardware.reset_gpio));
    hardware.busy_gpio = static_cast<int>(declare_parameter<int64_t>("busy_gpio", hardware.busy_gpio));
    hardware.dio1_gpio = static_cast<int>(declare_parameter<int64_t>("dio1_gpio", hardware.dio1_gpio));
    hardware.tx_enable_gpio = static_cast<int>(declare_parameter<int64_t>("tx_enable_gpio", hardware.tx_enable_gpio));
    hardware.rx_enable_gpio = static_cast<int>(declare_parameter<int64_t>("rx_enable_gpio", hardware.rx_enable_gpio));
    hardware.tx_enable_active_high =
        declare_parameter<bool>("tx_enable_active_high", hardware.tx_enable_active_high);
    hardware.rx_enable_active_high =
        declare_parameter<bool>("rx_enable_active_high", hardware.rx_enable_active_high);
    return hardware;
}

}  // namespace cubesat_radio
