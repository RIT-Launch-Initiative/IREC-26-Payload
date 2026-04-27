#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace cubesat_pi_io {

struct PowerReading {
  float bus_voltage_v;
  float current_a;
  float power_w;
};

class Ina260Driver {
public:
  Ina260Driver() = default;
  ~Ina260Driver();

  Ina260Driver(const Ina260Driver &) = delete;
  Ina260Driver &operator=(const Ina260Driver &) = delete;

  bool open(const std::string &device, uint8_t address);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  std::optional<PowerReading> read();

private:
  bool writeReg16(uint8_t reg, uint16_t val);
  bool readReg16(uint8_t reg, uint16_t &out);
  bool reset();
  bool writeDefaultConfig();

  int fd_{-1};
  uint8_t addr_{0};

  // Register map
  static constexpr uint8_t REG_CONF    = 0x00;
  static constexpr uint8_t REG_CURRENT = 0x01;
  static constexpr uint8_t REG_VBUS    = 0x02;
  static constexpr uint8_t REG_POWER   = 0x03;

  static constexpr uint16_t CONF_RESET = 1u << 15;

  // 1.25 mV/LSB, 1.25 mA/LSB, 10 mW/LSB per INA260 datasheet
  static constexpr float VOLTS_PER_LSB = 0.00125f;
  static constexpr float AMPS_PER_LSB  = 0.00125f;
  static constexpr float WATTS_PER_LSB = 0.01f;
};

}  // namespace cubesat_pi_io
