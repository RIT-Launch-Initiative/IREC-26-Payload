#pragma once

#include <cstdint>

namespace cubesat_radio {

struct RadioProfile {
    uint32_t frequency_hz{425450000};
    uint32_t bandwidth_hz{125000};
    uint8_t spreading_factor{12};
    uint8_t coding_rate{8};
    int8_t tx_power_dbm{22};
};

}  // namespace cubesat_radio
