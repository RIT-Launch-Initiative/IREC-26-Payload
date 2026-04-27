#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace cubesat_pi_io {

struct Lis3dhReading {
  float ax_mps2;
  float ay_mps2;
  float az_mps2;
};

class Lis3dhDriver {
public:
  Lis3dhDriver() = default;
  ~Lis3dhDriver();

  Lis3dhDriver(const Lis3dhDriver &) = delete;
  Lis3dhDriver &operator=(const Lis3dhDriver &) = delete;

  // Opens the I2C bus and verifies WHO_AM_I.
  bool open(const std::string &device, uint8_t address);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // sample_rate_hz: one of {1, 10, 25, 50, 100, 200, 400}. Other values
  // rejected. range_g: one of {2, 4, 8, 16}.
  bool configure(uint16_t sample_rate_hz, uint8_t range_g);

  std::optional<Lis3dhReading> read();

private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &out);
  bool readBlock(uint8_t reg, uint8_t *buf, size_t len);

  int fd_{-1};
  uint8_t addr_{0};

  // m/s² per raw LSB after right-shifting the 16-bit register value by 4.
  float mps2_per_lsb_{0.0f};
  bool configured_{false};

  // Register map (subset)
  static constexpr uint8_t REG_WHO_AM_I = 0x0F;
  static constexpr uint8_t REG_CTRL1 = 0x20;
  static constexpr uint8_t REG_CTRL4 = 0x23;
  static constexpr uint8_t REG_OUT_X_L = 0x28;

  static constexpr uint8_t WHO_AM_I_VAL = 0x33;

  // I2C multi-byte read requires the auto-increment bit in the subaddress
  static constexpr uint8_t AUTO_INC = 0x80;

  static constexpr float G_MPS2 = 9.80665f;
};

} // namespace cubesat_pi_io
