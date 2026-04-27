#pragma once

#include <cstdint>
#include <optional>
#include <string>

class Adxl375 {
public:
  struct AccelRaw {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  struct AccelMS2 {
    double x;
    double y;
    double z;
  };

  // Either 0x53 or 0x1D
  explicit Adxl375(std::string i2cDevPath, uint8_t addr = 0x1D);
  ~Adxl375();

  Adxl375(const Adxl375 &) = delete;
  Adxl375 &operator=(const Adxl375 &) = delete;
  Adxl375(Adxl375 &&) noexcept;
  Adxl375 &operator=(Adxl375 &&) noexcept;

  bool init();

  void close();

  bool isOpen() const { return fd >= 0; }

  bool setDataFormat(uint8_t dataFormat);
  bool setBwRate(uint8_t bwRate);
  bool setMeasurementMode(bool enable);
  bool setOffsets(int8_t ox, int8_t oy, int8_t oz);

  std::optional<AccelRaw> readRaw();
  std::optional<AccelMS2> readMS2();
  std::optional<uint8_t> readDeviceId();

private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &out);
  bool readRegs(uint8_t startReg, uint8_t *buff, size_t len);

private:
  std::string dev;
  uint8_t addr{0};
  int fd{-1};

  static constexpr uint8_t REG_DEVID = 0x00;
  static constexpr uint8_t REG_OFSX = 0x1E;
  static constexpr uint8_t REG_OFSY = 0x1F;
  static constexpr uint8_t REG_OFSZ = 0x20;
  static constexpr uint8_t REG_BW_RATE = 0x2C;
  static constexpr uint8_t REG_POWER_CTL = 0x2D;
  static constexpr uint8_t REG_DATA_FORMAT = 0x31;
  static constexpr uint8_t REG_DATAX0 = 0x32; // 0x32..0x37

  static constexpr uint8_t DEVID_VAL = 0xE5; // matches your Zephyr check

  static constexpr double MG_PER_LSB = 49.0; // mg/LSB
  static constexpr double G_MS2 = 9.80665;   // m/s^2
};
