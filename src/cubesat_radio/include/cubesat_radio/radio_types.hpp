#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cubesat_radio {

struct ReceivedPacket {
    std::vector<uint8_t> data;
    int16_t rssi_dbm{0};
    int8_t snr_db{0};
};

struct RadioHardwareConfig {
    std::string spi_device{"/dev/spidev0.0"};
    uint32_t spi_speed_hz{1000000};
    std::string gpio_chip_name{"gpiochip0"};
    int reset_gpio{-1};
    int busy_gpio{-1};
    int dio1_gpio{-1};
    int tx_enable_gpio{-1};
    int rx_enable_gpio{-1};
    bool tx_enable_active_high{true};
    bool rx_enable_active_high{true};
};

}  // namespace cubesat_radio
