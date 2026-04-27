#include "cubesat_pi_io/ina260_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace cubesat_pi_io {

namespace {

constexpr uint16_t bswap16(uint16_t v) {
  return static_cast<uint16_t>((v << 8) | (v >> 8));
}

} // namespace

Ina260Driver::~Ina260Driver() { close(); }

void Ina260Driver::close() {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

bool Ina260Driver::open(const std::string &device, uint8_t address) {
  close();

  fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    close();
    return false;
  }

  addr = address;

  if (!reset() || !writeDefaultConfig()) {
    close();
    return false;
  }
  return true;
}

bool Ina260Driver::reset() { return writeReg16(REG_CONF, CONF_RESET); }

bool Ina260Driver::writeDefaultConfig() {
  // Continuous voltage + current, 1100 µs conv time on each, 4-sample
  // averaging. Layout: [15:12]=0110 (fixed) | [11:9]=AVG | [8:6]=VBUS_CT |
  // [5:3]=ISH_CT | [2:0]=MODE
  constexpr uint16_t fixed_top = 0b0110u << 12;
  constexpr uint16_t avg_4 = 0b001u << 9;
  constexpr uint16_t vbus_ct = 0b100u << 6; // 1100 µs
  constexpr uint16_t ish_ct = 0b100u << 3;  // 1100 µs
  constexpr uint16_t mode_cont_both = 0b111u;
  constexpr uint16_t conf =
      fixed_top | avg_4 | vbus_ct | ish_ct | mode_cont_both;
  return writeReg16(REG_CONF, conf);
}

std::optional<PowerReading> Ina260Driver::read() {
  if (fd < 0) {
    return std::nullopt;
  }
  uint16_t v_raw = 0, i_raw = 0, p_raw = 0;
  if (!readReg16(REG_VBUS, v_raw))
    return std::nullopt;
  if (!readReg16(REG_CURRENT, i_raw))
    return std::nullopt;
  if (!readReg16(REG_POWER, p_raw))
    return std::nullopt;

  PowerReading r{};
  r.bus_voltage_v = static_cast<float>(v_raw) * VOLTS_PER_LSB;
  // Current register is signed two's complement
  r.current_a = static_cast<float>(static_cast<int16_t>(i_raw)) * AMPS_PER_LSB;
  r.power_w = static_cast<float>(p_raw) * WATTS_PER_LSB;
  return r;
}

bool Ina260Driver::writeReg16(uint8_t reg, uint16_t val) {
  uint16_t be = bswap16(val);
  uint8_t tx[3] = {reg, static_cast<uint8_t>(be >> 8),
                   static_cast<uint8_t>(be & 0xFF)};

  i2c_msg msg{};
  msg.addr = addr;
  msg.flags = 0;
  msg.len = sizeof(tx);
  msg.buf = tx;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = &msg;
  rdwr.nmsgs = 1;
  return ioctl(fd, I2C_RDWR, &rdwr) >= 0;
}

bool Ina260Driver::readReg16(uint8_t reg, uint16_t &out) {
  uint8_t rx[2]{};

  i2c_msg msgs[2]{};
  msgs[0].addr = addr;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &reg;

  msgs[1].addr = addr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = sizeof(rx);
  msgs[1].buf = rx;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = msgs;
  rdwr.nmsgs = 2;

  if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
    return false;
  }
  out = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
  return true;
}

} // namespace cubesat_pi_io
