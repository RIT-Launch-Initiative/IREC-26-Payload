#include "sensor_reader/adxl375.hpp"

#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

Adxl375::Adxl375(std::string i2cDevPath, uint8_t addr) : dev(std::move(i2cDevPath)), addr(addr) {}

Adxl375::~Adxl375() { close(); }

Adxl375::Adxl375(Adxl375 &&other) noexcept
    : dev(std::move(other.dev)), addr(other.addr), fd(other.fd) {
  other.fd = -1;
}

Adxl375 &Adxl375::operator=(Adxl375 &&other) noexcept {
  if (this == &other) {
    return *this;
  }
  
  close();
  dev = std::move(other.dev);
  addr = other.addr;
  fd = other.fd;
  other.fd = -1;
  return *this;
}

void Adxl375::close() {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

bool Adxl375::init() {
  if (fd >= 0)
    return true;

  fd = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0)
    return false;

  if (ioctl(fd, I2C_SLAVE, addr) < 0) {
    printf("I2C_SLAVE ioctl failed: %s\n", std::strerror(errno));
    close();
    return false;
  }

  auto devid = readDeviceId();
  if (!devid.has_value() || devid.value() != DEVID_VAL) {
    printf("ADXL375 device ID read failed or incorrect. Read: 0x%02X\n",
           devid.has_value() ? devid.value() : 0xFF);
    close();
    return false;
  }

  // Force bits 3, 1, and 0 in data format to set since reg default is 0
  if (!setDataFormat(0b00001011)) {
    printf("Failed to set data format register.\n");
    close();
    return false;
  }

  // Put into measurement mode
  if (!setMeasurementMode(true)) {
    printf("Failed to set measurement mode.\n");
    close();
    return false;
  }

  return true;
}

std::optional<uint8_t> Adxl375::readDeviceId() {
  uint8_t val = 0;
  if (!readReg(REG_DEVID, val))
    return std::nullopt;
  return val;
}

bool Adxl375::setDataFormat(uint8_t dataFormat) {
  return writeReg(REG_DATA_FORMAT, dataFormat);
}

bool Adxl375::setBwRate(uint8_t bwRate) {
  return writeReg(REG_BW_RATE, bwRate);
}

bool Adxl375::setMeasurementMode(bool enable) {
  uint8_t val = 0;
  if (!readReg(REG_POWER_CTL, val))
    return false;
  if (enable) {
    val |= (1u << 3);
  } else {
    val &= ~(1u << 3);
  }
  
  return writeReg(REG_POWER_CTL, val);
}

bool Adxl375::setOffsets(int8_t ox, int8_t oy, int8_t oz) {
  // Offsets are signed 8-bit
  return writeReg(REG_OFSX, static_cast<uint8_t>(ox)) &&
         writeReg(REG_OFSY, static_cast<uint8_t>(oy)) &&
         writeReg(REG_OFSZ, static_cast<uint8_t>(oz));
}

std::optional<Adxl375::AccelRaw> Adxl375::readRaw() {
  std::array<uint8_t, 6> buff{};
  if (!readRegs(REG_DATAX0, buff.data(), buff.size()))
    return std::nullopt;

  AccelRaw out{};
  out.x = static_cast<int16_t>((static_cast<uint16_t>(buff[1]) << 8) | buff[0]);
  out.y = static_cast<int16_t>((static_cast<uint16_t>(buff[3]) << 8) | buff[2]);
  out.z = static_cast<int16_t>((static_cast<uint16_t>(buff[5]) << 8) | buff[4]);
  return out;
}

std::optional<Adxl375::AccelMS2> Adxl375::readMS2() {
  auto raw = readRaw();
  if (!raw.has_value())
    return std::nullopt;

  // Copypasta-ing the conversion done in FSW ADXL375 driver
  constexpr double G_PER_LSB = (MG_PER_LSB / 1000.0); 
  AccelMS2 out{};
  out.x = static_cast<double>(raw->x) * G_MS2 * G_PER_LSB;
  out.y = static_cast<double>(raw->y) * G_MS2 * G_PER_LSB;
  out.z = static_cast<double>(raw->z) * G_MS2 * G_PER_LSB;
  return out;
}

bool Adxl375::writeReg(uint8_t reg, uint8_t val) {
  uint8_t data[2] = {reg, val};

  i2c_msg msg{};
  msg.addr = addr;
  msg.flags = 0;
  msg.len = sizeof(data);
  msg.buf = data;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = &msg;
  rdwr.nmsgs = 1;

  if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
    return false;
  }

  return true;
}

bool Adxl375::readReg(uint8_t reg, uint8_t &out) {
  return readRegs(reg, &out, 1);
}

bool Adxl375::readRegs(uint8_t startReg, uint8_t *buff, size_t len) {
  i2c_msg msgs[2]{};

  msgs[0].addr = addr;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &startReg;

  msgs[1].addr = addr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<__u16>(len);
  msgs[1].buf = buff;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = msgs;
  rdwr.nmsgs = 2;

  if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
    return false;
  }

  return true;
}
