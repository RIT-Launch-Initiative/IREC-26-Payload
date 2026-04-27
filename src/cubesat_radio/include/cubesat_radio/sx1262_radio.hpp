#pragma once

#include <optional>
#include <vector>

#include "cubesat_radio/radio_profile.hpp"
#include "cubesat_radio/radio_types.hpp"

namespace cubesat_radio {

class Sx1262Radio {
public:
    explicit Sx1262Radio(RadioHardwareConfig hardware = {});
    ~Sx1262Radio();

    Sx1262Radio(const Sx1262Radio&) = delete;
    Sx1262Radio& operator=(const Sx1262Radio&) = delete;

    bool open();
    bool configure(const RadioProfile& profile);
    bool send(const std::vector<uint8_t>& data);
    std::optional<ReceivedPacket> receive();
    bool setReceiveMode();
    bool setSleepMode();

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace cubesat_radio
