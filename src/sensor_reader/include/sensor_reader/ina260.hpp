// ina260.hpp
#pragma once

#include <cstdint>
#include <string>
#include <optional>

class Ina260 {
public:
    enum class Mode : uint8_t {
        TRIG_OFF     = 0b000,
        TRIG_CURRENT = 0b001,
        TRIG_VOLTAGE = 0b010,
        TRIG_BOTH    = 0b011,
        CONT_OFF     = 0b100,
        CONT_CURRENT = 0b101,
        CONT_VOLTAGE = 0b110,
        CONT_BOTH    = 0b111,
    };

    enum class Average : uint8_t {
        AVG_1    = 0b000,
        AVG_4    = 0b001,
        AVG_16   = 0b010,
        AVG_64   = 0b011,
        AVG_128  = 0b100,
        AVG_256  = 0b101,
        AVG_512  = 0b110,
        AVG_1024 = 0b111,
    };

    enum class ConvTime : uint8_t {
        CT_140US  = 0b000,
        CT_204US  = 0b001,
        CT_332US  = 0b010,
        CT_588US  = 0b011,
        CT_1100US = 0b100,
        CT_2116US = 0b101,
        CT_4156US = 0b110,
        CT_8244US = 0b111,
    };

    struct Config {
        Average average;
        ConvTime voltage_conv_time;
        ConvTime current_conv_time;
        Mode mode;

        Config() 
            : average(Average::AVG_1),
              voltage_conv_time(ConvTime::CT_1100US),
              current_conv_time(ConvTime::CT_1100US),
              mode(Mode::CONT_BOTH) {}
    };

    explicit Ina260(std::string i2cDevPath, uint8_t addr = 0x40, Config cfg = Config());
    ~Ina260();

    Ina260(const Ina260&) = delete;
    Ina260& operator=(const Ina260&) = delete;
    Ina260(Ina260&&) noexcept;
    Ina260& operator=(Ina260&&) noexcept;

    bool init();
    void close();
    bool isOpen() const { return fd >= 0; }

    bool reset();
    bool writeConfig(const Config& cfg);
    std::optional<uint16_t> readManufacturerId();
    std::optional<uint16_t> readDieId();

    std::optional<double> readBusVoltage_V(); // V
    std::optional<double> readCurrent_A();    // A
    std::optional<double> readPower_W();      // W

private:
    bool writeReg16(uint8_t reg, uint16_t valHost);
    bool readReg16(uint8_t reg, uint16_t& outHost);

private:
    std::string dev;
    uint8_t addr{0};
    int fd{-1};
    Config cfg{};

private:
    static constexpr uint8_t REG_CONF    = 0x00;
    static constexpr uint8_t REG_CURRENT = 0x01;
    static constexpr uint8_t REG_VBUS    = 0x02;
    static constexpr uint8_t REG_POWER   = 0x03;
    static constexpr uint8_t REG_MAN_ID  = 0xFE;
    static constexpr uint8_t REG_DIE_ID  = 0xFF;

    static constexpr uint16_t CONF_RST = 1u << 15;
    static constexpr uint16_t CONF_REQUIRED_TOP_BITS = 0b0110; // goes in [15:12] as 0b0110xxxx

    static constexpr double VOLTS_PER_LSB = 0.00125; // 1.25 mV/LSB
    static constexpr double AMPS_PER_LSB  = 0.00125; // 1.25 mA/LSB
    static constexpr double WATTS_PER_LSB = 0.01;    // 10 mW/LSB
};
