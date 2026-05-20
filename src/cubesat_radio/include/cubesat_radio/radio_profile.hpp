#pragma once

#include <cstdint>

namespace cubesat_radio {

struct RadioProfile {
    uint32_t frequency_hz{915000000};
    uint32_t bandwidth_hz{125000};
    uint8_t spreading_factor{7};
    uint8_t coding_rate{5};
    int8_t tx_power_dbm{28};
};

}  // namespace cubesat_radio
