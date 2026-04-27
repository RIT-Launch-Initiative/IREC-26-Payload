#include "sx126x_linux_hal.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

extern "C" {
#include "sx126x_hal.h"
}

namespace cubesat_radio {

namespace {

using Clock = std::chrono::steady_clock;

bool request_output(gpiod_chip* chip, int line_number, const char* consumer, int initial_value, gpiod_line*& out_line) {
    if (line_number < 0) {
        out_line = nullptr;
        return true;
    }

    out_line = gpiod_chip_get_line(chip, line_number);
    if (out_line == nullptr) {
        return false;
    }
    if (gpiod_line_request_output(out_line, consumer, initial_value) != 0) {
        gpiod_line_release(out_line);
        out_line = nullptr;
        return false;
    }
    return true;
}

bool request_input(gpiod_chip* chip, int line_number, const char* consumer, gpiod_line*& out_line) {
    if (line_number < 0) {
        out_line = nullptr;
        return true;
    }

    out_line = gpiod_chip_get_line(chip, line_number);
    if (out_line == nullptr) {
        return false;
    }
    if (gpiod_line_request_input(out_line, consumer) != 0) {
        gpiod_line_release(out_line);
        out_line = nullptr;
        return false;
    }
    return true;
}

bool set_optional_output(gpiod_line* line, bool active_high, bool enabled) {
    if (line == nullptr) {
        return true;
    }
    const int value = enabled ? (active_high ? 1 : 0) : (active_high ? 0 : 1);
    return gpiod_line_set_value(line, value) == 0;
}

}  // namespace

bool wait_for_busy_clear(Sx126xLinuxHalContext& context, unsigned int timeout_ms) {
    if (context.busy == nullptr) {
        return true;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (Clock::now() < deadline) {
        const int value = gpiod_line_get_value(context.busy);
        if (value < 0) {
            return false;
        }
        if (value == 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return false;
}

bool set_rf_switch_rx(Sx126xLinuxHalContext& context) {
    return set_optional_output(context.tx_enable, context.tx_enable_active_high, false) &&
           set_optional_output(context.rx_enable, context.rx_enable_active_high, true);
}

bool set_rf_switch_tx(Sx126xLinuxHalContext& context) {
    return set_optional_output(context.rx_enable, context.rx_enable_active_high, false) &&
           set_optional_output(context.tx_enable, context.tx_enable_active_high, true);
}

bool set_rf_switch_idle(Sx126xLinuxHalContext& context) {
    return set_optional_output(context.tx_enable, context.tx_enable_active_high, false) &&
           set_optional_output(context.rx_enable, context.rx_enable_active_high, false);
}

bool open_linux_hal(Sx126xLinuxHalContext& context, const std::string& spi_device, uint32_t spi_speed_hz,
                    const std::string& gpio_chip_name, int reset_gpio, int busy_gpio, int dio1_gpio, int tx_enable_gpio,
                    int rx_enable_gpio, bool tx_enable_active_high, bool rx_enable_active_high) {
    close_linux_hal(context);

    context.spi_fd = ::open(spi_device.c_str(), O_RDWR | O_CLOEXEC);
    if (context.spi_fd < 0) {
        return false;
    }

    context.spi_speed_hz = spi_speed_hz;
    context.tx_enable_active_high = tx_enable_active_high;
    context.rx_enable_active_high = rx_enable_active_high;

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(context.spi_fd, SPI_IOC_WR_MODE, &mode) != 0 || ioctl(context.spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) != 0 ||
        ioctl(context.spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &context.spi_speed_hz) != 0) {
        close_linux_hal(context);
        return false;
    }

    context.chip = gpiod_chip_open_by_name(gpio_chip_name.c_str());
    if (context.chip == nullptr) {
        close_linux_hal(context);
        return false;
    }

    if (!request_output(context.chip, reset_gpio, "sx1262_reset", 1, context.reset) ||
        !request_input(context.chip, busy_gpio, "sx1262_busy", context.busy) ||
        !request_input(context.chip, dio1_gpio, "sx1262_dio1", context.dio1) ||
        !request_output(context.chip, tx_enable_gpio, "sx1262_tx_enable", tx_enable_active_high ? 0 : 1,
                        context.tx_enable) ||
        !request_output(context.chip, rx_enable_gpio, "sx1262_rx_enable", rx_enable_active_high ? 0 : 1,
                        context.rx_enable)) {
        close_linux_hal(context);
        return false;
    }

    if (!set_rf_switch_idle(context)) {
        close_linux_hal(context);
        return false;
    }

    return true;
}

void close_linux_hal(Sx126xLinuxHalContext& context) {
    if (context.reset != nullptr) {
        gpiod_line_release(context.reset);
        context.reset = nullptr;
    }
    if (context.busy != nullptr) {
        gpiod_line_release(context.busy);
        context.busy = nullptr;
    }
    if (context.dio1 != nullptr) {
        gpiod_line_release(context.dio1);
        context.dio1 = nullptr;
    }
    if (context.tx_enable != nullptr) {
        gpiod_line_release(context.tx_enable);
        context.tx_enable = nullptr;
    }
    if (context.rx_enable != nullptr) {
        gpiod_line_release(context.rx_enable);
        context.rx_enable = nullptr;
    }
    if (context.chip != nullptr) {
        gpiod_chip_close(context.chip);
        context.chip = nullptr;
    }
    if (context.spi_fd >= 0) {
        ::close(context.spi_fd);
        context.spi_fd = -1;
    }
}

}  // namespace cubesat_radio

extern "C" {

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    auto* hal = static_cast<cubesat_radio::Sx126xLinuxHalContext*>(const_cast<void*>(context));
    if (hal == nullptr || hal->spi_fd < 0 || !cubesat_radio::wait_for_busy_clear(*hal, hal->busy_timeout_ms)) {
        return SX126X_HAL_STATUS_ERROR;
    }

    spi_ioc_transfer transfer[2]{};
    transfer[0].tx_buf = reinterpret_cast<unsigned long>(command);
    transfer[0].len = command_length;
    transfer[0].speed_hz = hal->spi_speed_hz;
    transfer[0].bits_per_word = 8;

    transfer[1].tx_buf = reinterpret_cast<unsigned long>(data);
    transfer[1].len = data_length;
    transfer[1].speed_hz = hal->spi_speed_hz;
    transfer[1].bits_per_word = 8;

    if (ioctl(hal->spi_fd, SPI_IOC_MESSAGE(2), transfer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }

    return cubesat_radio::wait_for_busy_clear(*hal, hal->busy_timeout_ms) ? SX126X_HAL_STATUS_OK
                                                                           : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    auto* hal = static_cast<cubesat_radio::Sx126xLinuxHalContext*>(const_cast<void*>(context));
    if (hal == nullptr || hal->spi_fd < 0 || !cubesat_radio::wait_for_busy_clear(*hal, hal->busy_timeout_ms)) {
        return SX126X_HAL_STATUS_ERROR;
    }

    spi_ioc_transfer transfer[2]{};
    transfer[0].tx_buf = reinterpret_cast<unsigned long>(command);
    transfer[0].len = command_length;
    transfer[0].speed_hz = hal->spi_speed_hz;
    transfer[0].bits_per_word = 8;

    transfer[1].rx_buf = reinterpret_cast<unsigned long>(data);
    transfer[1].len = data_length;
    transfer[1].speed_hz = hal->spi_speed_hz;
    transfer[1].bits_per_word = 8;

    if (ioctl(hal->spi_fd, SPI_IOC_MESSAGE(2), transfer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }

    return cubesat_radio::wait_for_busy_clear(*hal, hal->busy_timeout_ms) ? SX126X_HAL_STATUS_OK
                                                                           : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    auto* hal = static_cast<cubesat_radio::Sx126xLinuxHalContext*>(const_cast<void*>(context));
    if (hal == nullptr || hal->reset == nullptr) {
        return SX126X_HAL_STATUS_ERROR;
    }

    if (gpiod_line_set_value(hal->reset, 0) != 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    usleep(2000);
    if (gpiod_line_set_value(hal->reset, 1) != 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    usleep(5000);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    auto* hal = static_cast<cubesat_radio::Sx126xLinuxHalContext*>(const_cast<void*>(context));
    if (hal == nullptr || hal->spi_fd < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }

    uint8_t dummy = 0x00;
    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<unsigned long>(&dummy);
    transfer.len = 1;
    transfer.speed_hz = hal->spi_speed_hz;
    transfer.bits_per_word = 8;

    if (ioctl(hal->spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }

    return cubesat_radio::wait_for_busy_clear(*hal, hal->busy_timeout_ms) ? SX126X_HAL_STATUS_OK
                                                                           : SX126X_HAL_STATUS_ERROR;
}

}  // extern "C"
