#include "sensor_reader/ina260.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

static inline uint16_t bswap16(uint16_t v) {
    return static_cast<uint16_t>((v << 8) | (v >> 8));
}

Ina260::Ina260(std::string i2cDevPath, uint8_t addr, Config cfg)
    : dev(std::move(i2cDevPath)), addr(addr), cfg(cfg) {}

Ina260::~Ina260() { close(); }

Ina260::Ina260(Ina260&& other) noexcept
    : dev(std::move(other.dev)), addr(other.addr), fd(other.fd), cfg(other.cfg) {
    other.fd = -1;
}

Ina260& Ina260::operator=(Ina260&& other) noexcept {
    if (this == &other) return *this;
    close();
    dev = std::move(other.dev);
    addr = other.addr;
    fd = other.fd;
    cfg = other.cfg;
    other.fd = -1;
    return *this;
}

void Ina260::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool Ina260::init() {
    if (fd >= 0) return true;

    fd = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) return false;

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        close();
        return false;
    }

    if (!reset()) {
        close();
        return false;
    }

    if (!writeConfig(cfg)) {
        close();
        return false;
    }

    return true;
}

bool Ina260::reset() {
    return writeReg16(REG_CONF, CONF_RST);
}

bool Ina260::writeConfig(const Config& cfg) {
    const uint16_t conf =
        static_cast<uint16_t>(CONF_REQUIRED_TOP_BITS << 12) |
        static_cast<uint16_t>(static_cast<uint8_t>(cfg.average) << 9) |
        static_cast<uint16_t>(static_cast<uint8_t>(cfg.voltage_conv_time) << 6) |
        static_cast<uint16_t>(static_cast<uint8_t>(cfg.current_conv_time) << 3) |
        static_cast<uint16_t>(static_cast<uint8_t>(cfg.mode));

    this->cfg = cfg;
    return writeReg16(REG_CONF, conf);
}

std::optional<uint16_t> Ina260::readManufacturerId() {
    uint16_t val = 0;
    if (!readReg16(REG_MAN_ID, val)) return std::nullopt;
    return val;
}

std::optional<uint16_t> Ina260::readDieId() {
    uint16_t val = 0;
    if (!readReg16(REG_DIE_ID, val)) return std::nullopt;
    return val;
}

std::optional<double> Ina260::readBusVoltage_V() {
    uint16_t r = 0;
    if (!readReg16(REG_VBUS, r)) return std::nullopt;
    return static_cast<double>(r) * VOLTS_PER_LSB;
}

std::optional<double> Ina260::readCurrent_A() {
    uint16_t r = 0;
    if (!readReg16(REG_CURRENT, r)) return std::nullopt;
    return static_cast<double>(r) * AMPS_PER_LSB;
}

std::optional<double> Ina260::readPower_W() {
    uint16_t val = 0;
    if (!readReg16(REG_POWER, val)) return std::nullopt;
    return static_cast<double>(val) * WATTS_PER_LSB;
}

bool Ina260::writeReg16(uint8_t reg, uint16_t valHost) {
    // INA260 uses big endian register 
    uint16_t be = bswap16(valHost);
    uint8_t tx[3] = { reg, static_cast<uint8_t>(be >> 8), static_cast<uint8_t>(be & 0xFF) };

    i2c_msg msg{};
    msg.addr = addr;
    msg.flags = 0;
    msg.len = sizeof(tx);
    msg.buf = tx;

    i2c_rdwr_ioctl_data rdwr{};
    rdwr.msgs  = &msg;
    rdwr.nmsgs = 1;

    return ioctl(fd, I2C_RDWR, &rdwr) >= 0;
}

bool Ina260::readReg16(uint8_t reg, uint16_t& outHost) {
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

    if (ioctl(fd, I2C_RDWR, &rdwr) < 0) return false;

    // rx is big-endian convert to host
    uint16_t be = static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8) | rx[1]);
    outHost = bswap16(be);
    return true;
}
