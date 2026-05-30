#pragma once
#include <gpiod.h>
#include <string>

namespace cubesat_pi_io {
class BuzzerDriver {

  public:
    BuzzerDriver();
    bool open(const std::string &chip_name, int line_num);
    void close();

    void set_code(uint32_t code, uint32_t next_repeat_code);
    void onTimer();

    gpiod_chip *chip{nullptr};
    gpiod_line *buzzer{nullptr};

    uint32_t active_code = 0;
    uint32_t index = 0;
    int repeat_count = false;
};

} // namespace cubesat_io_node