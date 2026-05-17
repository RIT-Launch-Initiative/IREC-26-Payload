#pragma once

#include <cstdint>
#include <string>

#include <gpiod.h>

namespace cubesat_radio {

struct Sx126xLinuxHalContext {
    int spi_fd{-1};
    uint32_t spi_speed_hz{1000000};
    gpiod_chip* chip{nullptr};
    gpiod_line* reset{nullptr};
    gpiod_line* busy{nullptr};
    gpiod_line* dio1{nullptr};
    gpiod_line* tx_enable{nullptr};
    gpiod_line* rx_enable{nullptr};
    bool tx_enable_active_high{true};
    bool rx_enable_active_high{true};
    unsigned int busy_timeout_ms{100};
};

bool open_linux_hal(Sx126xLinuxHalContext& context, const std::string& spi_device, uint32_t spi_speed_hz,
                    const std::string& gpio_chip_name, int reset_gpio, int busy_gpio, int dio1_gpio, int tx_enable_gpio,
                    int rx_enable_gpio, bool tx_enable_active_high, bool rx_enable_active_high);

void close_linux_hal(Sx126xLinuxHalContext& context);

bool set_rf_switch_rx(Sx126xLinuxHalContext& context);
bool set_rf_switch_tx(Sx126xLinuxHalContext& context);
bool set_rf_switch_idle(Sx126xLinuxHalContext& context);
bool wait_for_busy_clear(Sx126xLinuxHalContext& context, unsigned int timeout_ms);
bool wait_for_dio1_rising(Sx126xLinuxHalContext& context, int timeout_ms);

}  // namespace cubesat_radio
