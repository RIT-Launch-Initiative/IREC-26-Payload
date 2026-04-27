#include "cubesat_radio/sx1262_radio.hpp"

#include <chrono>
#include <mutex>
#include <thread>

#include "sx126x_linux_hal.hpp"

extern "C" {
#include "sx126x.h"
#include "sx126x_hal.h"
}

namespace cubesat_radio {

namespace {

using Clock = std::chrono::steady_clock;

constexpr uint16_t kPreambleLength = 8;
constexpr uint32_t kTxTimeoutMs = 3000;

sx126x_lora_bw_t to_bandwidth_enum(uint32_t bandwidth_hz) {
  if (bandwidth_hz >= 500000) {
    return SX126X_LORA_BW_500;
  }
  if (bandwidth_hz >= 250000) {
    return SX126X_LORA_BW_250;
  }
  if (bandwidth_hz >= 125000) {
    return SX126X_LORA_BW_125;
  }
  if (bandwidth_hz >= 62500) {
    return SX126X_LORA_BW_062;
  }
  if (bandwidth_hz >= 41700) {
    return SX126X_LORA_BW_041;
  }
  if (bandwidth_hz >= 31250) {
    return SX126X_LORA_BW_031;
  }
  if (bandwidth_hz >= 20800) {
    return SX126X_LORA_BW_020;
  }
  if (bandwidth_hz >= 15600) {
    return SX126X_LORA_BW_015;
  }
  return SX126X_LORA_BW_010;
}

sx126x_lora_cr_t to_coding_rate_enum(uint8_t coding_rate) {
  switch (coding_rate) {
  case 5:
    return SX126X_LORA_CR_4_5;
  case 6:
    return SX126X_LORA_CR_4_6;
  case 7:
    return SX126X_LORA_CR_4_7;
  case 8:
    return SX126X_LORA_CR_4_8;
  default:
    return SX126X_LORA_CR_4_5;
  }
}

uint8_t compute_ldro(uint8_t spreading_factor, uint32_t bandwidth_hz) {
  if (spreading_factor >= 11 && bandwidth_hz <= 125000) {
    return 1;
  }
  if (spreading_factor == 10 && bandwidth_hz <= 62500) {
    return 1;
  }
  return 0;
}

bool wait_for_irq(Sx126xLinuxHalContext &hal, sx126x_irq_mask_t wanted,
                  uint32_t timeout_ms, sx126x_irq_mask_t &seen) {
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  seen = 0;

  while (Clock::now() < deadline) {
    if (sx126x_get_and_clear_irq_status(&hal, &seen) != SX126X_STATUS_OK) {
      return false;
    }
    if ((seen & wanted) != 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return true;
}

} // namespace

struct Sx1262Radio::Impl {
  explicit Impl(RadioHardwareConfig config_in) : config(config_in) {}

  RadioHardwareConfig config;
  RadioProfile profile;
  Sx126xLinuxHalContext hal;
  bool is_open{false};
  bool is_configured{false};
  std::mutex mutex;
};

Sx1262Radio::Sx1262Radio(RadioHardwareConfig hardware)
    : impl(new Impl(hardware)) {}

Sx1262Radio::~Sx1262Radio() {
  close_linux_hal(impl->hal);
  delete impl;
}

bool Sx1262Radio::open() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->is_open) {
    return true;
  }

  if (!open_linux_hal(impl->hal, impl->config.spi_device,
                      impl->config.spi_speed_hz, impl->config.gpio_chip_name,
                      impl->config.reset_gpio, impl->config.busy_gpio,
                      impl->config.dio1_gpio, impl->config.tx_enable_gpio,
                      impl->config.rx_enable_gpio,
                      impl->config.tx_enable_active_high,
                      impl->config.rx_enable_active_high)) {
    return false;
  }

  if (sx126x_hal_reset(&impl->hal) != SX126X_HAL_STATUS_OK ||
      sx126x_hal_wakeup(&impl->hal) != SX126X_HAL_STATUS_OK ||
      sx126x_set_standby(&impl->hal, SX126X_STANDBY_CFG_RC) !=
          SX126X_STATUS_OK) {
    close_linux_hal(impl->hal);
    return false;
  }

  sx126x_pa_cfg_params_t pa_config{0x04, 0x07, 0x00, 0x01};
  if (sx126x_set_pa_cfg(&impl->hal, &pa_config) != SX126X_STATUS_OK) {
    close_linux_hal(impl->hal);
    return false;
  }

  impl->is_open = true;
  return true;
}

bool Sx1262Radio::configure(const RadioProfile &profile) {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->is_open) {
    return false;
  }

  impl->profile = profile;

  if (sx126x_set_standby(&impl->hal, SX126X_STANDBY_CFG_RC) !=
          SX126X_STATUS_OK ||
      sx126x_set_pkt_type(&impl->hal, SX126X_PKT_TYPE_LORA) !=
          SX126X_STATUS_OK ||
      sx126x_set_tx_params(&impl->hal, profile.tx_power_dbm,
                           SX126X_RAMP_200_US) != SX126X_STATUS_OK ||
      sx126x_set_rf_freq(&impl->hal, profile.frequency_hz) !=
          SX126X_STATUS_OK) {
    return false;
  }

  const sx126x_mod_params_lora_t mod_params{
      static_cast<sx126x_lora_sf_t>(profile.spreading_factor),
      to_bandwidth_enum(profile.bandwidth_hz),
      to_coding_rate_enum(profile.coding_rate),
      compute_ldro(profile.spreading_factor, profile.bandwidth_hz),
  };
  if (sx126x_set_lora_mod_params(&impl->hal, &mod_params) != SX126X_STATUS_OK) {
    return false;
  }

  const sx126x_pkt_params_lora_t pkt_params{
      kPreambleLength, SX126X_LORA_PKT_EXPLICIT, 0xFF, true, false,
  };
  if (sx126x_set_lora_pkt_params(&impl->hal, &pkt_params) != SX126X_STATUS_OK ||
      sx126x_set_buffer_base_address(&impl->hal, 0x00, 0x00) !=
          SX126X_STATUS_OK ||
      sx126x_set_dio_irq_params(&impl->hal,
                                SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |
                                    SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |
                                    SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                0, 0) != SX126X_STATUS_OK ||
      sx126x_clear_irq_status(&impl->hal, SX126X_IRQ_ALL) != SX126X_STATUS_OK) {
    return false;
  }

  impl->is_configured = true;
  return true;
}

bool Sx1262Radio::send(const std::vector<uint8_t> &data) {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->is_configured || data.empty() || data.size() > 255) {
    return false;
  }

  if (!set_rf_switch_tx(impl->hal)) {
    return false;
  }

  const sx126x_pkt_params_lora_t pkt_params{
      kPreambleLength,
      SX126X_LORA_PKT_EXPLICIT,
      static_cast<uint8_t>(data.size()),
      true,
      false,
  };
  if (sx126x_set_lora_pkt_params(&impl->hal, &pkt_params) != SX126X_STATUS_OK ||
      sx126x_set_buffer_base_address(&impl->hal, 0x00, 0x00) !=
          SX126X_STATUS_OK ||
      sx126x_write_buffer(&impl->hal, 0x00, data.data(),
                          static_cast<uint8_t>(data.size())) !=
          SX126X_STATUS_OK ||
      sx126x_clear_irq_status(&impl->hal, SX126X_IRQ_ALL) != SX126X_STATUS_OK ||
      sx126x_set_tx(&impl->hal, kTxTimeoutMs) != SX126X_STATUS_OK) {
    set_rf_switch_idle(impl->hal);
    return false;
  }

  sx126x_irq_mask_t irq = 0;
  const bool got_irq =
      wait_for_irq(impl->hal, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                   kTxTimeoutMs + 100, irq);
  const bool success = got_irq && ((irq & SX126X_IRQ_TX_DONE) != 0);

  set_rf_switch_rx(impl->hal);
  sx126x_set_rx(&impl->hal, SX126X_RX_CONTINUOUS);
  return success;
}

std::optional<ReceivedPacket> Sx1262Radio::receive() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->is_configured) {
    return std::nullopt;
  }

  sx126x_irq_mask_t irq = 0;
  if (sx126x_get_and_clear_irq_status(&impl->hal, &irq) != SX126X_STATUS_OK) {
    return std::nullopt;
  }
  if ((irq & SX126X_IRQ_RX_DONE) == 0) {
    return std::nullopt;
  }

  sx126x_rx_buffer_status_t rx_status{};
  sx126x_pkt_status_lora_t pkt_status{};
  if (sx126x_get_rx_buffer_status(&impl->hal, &rx_status) != SX126X_STATUS_OK ||
      sx126x_get_lora_pkt_status(&impl->hal, &pkt_status) != SX126X_STATUS_OK ||
      rx_status.pld_len_in_bytes == 0) {
    return std::nullopt;
  }

  ReceivedPacket packet;
  packet.data.resize(rx_status.pld_len_in_bytes);
  if (sx126x_read_buffer(&impl->hal, rx_status.buffer_start_pointer,
                         packet.data.data(),
                         rx_status.pld_len_in_bytes) != SX126X_STATUS_OK) {
    return std::nullopt;
  }
  packet.rssi_dbm = pkt_status.rssi_pkt_in_dbm;
  packet.snr_db = pkt_status.snr_pkt_in_db;

  sx126x_clear_irq_status(&impl->hal, SX126X_IRQ_ALL);
  sx126x_set_rx(&impl->hal, SX126X_RX_CONTINUOUS);
  return packet;
}

bool Sx1262Radio::setReceiveMode() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->is_configured || !set_rf_switch_rx(impl->hal)) {
    return false;
  }
  return sx126x_set_rx(&impl->hal, SX126X_RX_CONTINUOUS) == SX126X_STATUS_OK;
}

bool Sx1262Radio::setSleepMode() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->is_open || !set_rf_switch_idle(impl->hal)) {
    return false;
  }

  return sx126x_set_sleep(&impl->hal, SX126X_SLEEP_CFG_WARM_START) ==
         SX126X_STATUS_OK;
}

bool Sx1262Radio::waitForInterrupt(std::chrono::milliseconds timeout) {
  if (timeout.count() < 0) {
    timeout = std::chrono::milliseconds(0);
  }

  {
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->is_configured) {
      return false;
    }
  }

  return wait_for_dio1_rising(impl->hal, static_cast<int>(timeout.count()));
}

} // namespace cubesat_radio
