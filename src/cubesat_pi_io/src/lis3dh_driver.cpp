#include "cubesat_pi_io/lis3dh_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace cubesat_pi_io {

namespace {

// Output Data Rate codes for CTRL_REG1[7:4], normal-mode operation.
// Returns -1 if the rate is not supported.
int odrCode(uint16_t hz) {
  switch (hz) {
  case 1:
    return 0x1;
  case 10:
    return 0x2;
  case 25:
    return 0x3;
  case 50:
    return 0x4;
  case 100:
    return 0x5;
  case 200:
    return 0x6;
  case 400:
    return 0x7;
  default:
    return -1;
  }
}

// HR=1 (12-bit) sensitivity in mg/LSB, after the raw 16-bit register has been
// right-shifted by 4 to extract the 12 significant bits.
// Source: LIS3DH datasheet, Table 4.
float mgPerLsb(uint8_t range_g) {
  switch (range_g) {
  case 2:
    return 1.0f;
  case 4:
    return 2.0f;
  case 8:
    return 4.0f;
  case 16:
    return 12.0f; // datasheet quirk — not 8
  default:
    return 0.0f;
  }
}

// FS bits in CTRL_REG4[5:4]
int fsCode(uint8_t range_g) {
  switch (range_g) {
  case 2:
    return 0x0;
  case 4:
    return 0x1;
  case 8:
    return 0x2;
  case 16:
    return 0x3;
  default:
    return -1;
  }
}

} // namespace

Lis3dhDriver::~Lis3dhDriver() { close(); }

void Lis3dhDriver::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  configured_ = false;
}

bool Lis3dhDriver::open(const std::string &device, uint8_t address) {
  close();

  int fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    ::close(fd);
    return false;
  }

  fd_ = fd;
  addr_ = address;

  uint8_t who = 0;
  if (!readReg(REG_WHO_AM_I, who) || who != WHO_AM_I_VAL) {
    close();
    return false;
  }
  return true;
}

bool Lis3dhDriver::configure(uint16_t sample_rate_hz, uint8_t range_g) {
  if (fd_ < 0) {
    return false;
  }
  int odr = odrCode(sample_rate_hz);
  int fs = fsCode(range_g);
  if (odr < 0 || fs < 0) {
    return false;
  }

  // CTRL_REG1: ODR[7:4] | LPen=0 (normal mode) | Zen=Yen=Xen=1
  const uint8_t ctrl1 = static_cast<uint8_t>((odr << 4) | 0x07);
  // CTRL_REG4: BDU=1 | BLE=0 | FS[5:4] | HR=1 | ST=00 | SIM=0
  const uint8_t ctrl4 = static_cast<uint8_t>(0x80 | (fs << 4) | 0x08);

  if (!writeReg(REG_CTRL1, ctrl1))
    return false;
  if (!writeReg(REG_CTRL4, ctrl4))
    return false;

  mps2_per_lsb_ = mgPerLsb(range_g) * (G_MPS2 / 1000.0f);
  configured_ = true;
  return true;
}

std::optional<Lis3dhReading> Lis3dhDriver::read() {
  if (fd_ < 0 || !configured_) {
    return std::nullopt;
  }

  uint8_t buf[6]{};
  if (!readBlock(REG_OUT_X_L, buf, sizeof(buf))) {
    return std::nullopt;
  }

  // Output is 16-bit left-justified two's complement; shift right by 4 to
  // recover the 12-bit signed value. Signed shift on int16_t preserves sign.
  auto raw = [&](int idx) -> int16_t {
    int16_t v = static_cast<int16_t>(buf[idx] | (buf[idx + 1] << 8));
    return static_cast<int16_t>(v >> 4);
  };

  Lis3dhReading r{};
  r.ax_mps2 = static_cast<float>(raw(0)) * mps2_per_lsb_;
  r.ay_mps2 = static_cast<float>(raw(2)) * mps2_per_lsb_;
  r.az_mps2 = static_cast<float>(raw(4)) * mps2_per_lsb_;
  return r;
}

bool Lis3dhDriver::writeReg(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {reg, val};

  i2c_msg msg{};
  msg.addr = addr_;
  msg.flags = 0;
  msg.len = sizeof(tx);
  msg.buf = tx;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = &msg;
  rdwr.nmsgs = 1;
  return ioctl(fd_, I2C_RDWR, &rdwr) >= 0;
}

bool Lis3dhDriver::readReg(uint8_t reg, uint8_t &out) {
  return readBlock(reg, &out, 1);
}

bool Lis3dhDriver::readBlock(uint8_t reg, uint8_t *buf, size_t len) {
  uint8_t sub = static_cast<uint8_t>(reg | (len > 1 ? AUTO_INC : 0));

  i2c_msg msgs[2]{};
  msgs[0].addr = addr_;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &sub;

  msgs[1].addr = addr_;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<uint16_t>(len);
  msgs[1].buf = buf;

  i2c_rdwr_ioctl_data rdwr{};
  rdwr.msgs = msgs;
  rdwr.nmsgs = 2;
  return ioctl(fd_, I2C_RDWR, &rdwr) >= 0;
}

} // namespace cubesat_pi_io
