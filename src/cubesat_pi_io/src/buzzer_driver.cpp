#include "cubesat_pi_io/buzzer_driver.hpp"

namespace cubesat_pi_io {
BuzzerDriver::BuzzerDriver() {}


bool request_output(gpiod_chip* chip, int line_number, const char* consumer, int initial_value, gpiod_line*& out_line) {
    if (line_number < 0) {
        out_line = nullptr;
        return true;
    }

    out_line = gpiod_chip_get_line(chip, line_number);
    if (out_line == nullptr) {
        return false;
    }
    if (gpiod_line_request_output(out_line, consumer, initial_value) != 0) {
        gpiod_line_release(out_line);
        out_line = nullptr;
        return false;
    }
    return true;
}

void BuzzerDriver::set_code(uint32_t code, uint32_t next_repeat_count){
    active_code = code;
    repeat_count = next_repeat_count;
}

bool BuzzerDriver::open(const std::string &chip_name, int line_num) {
    chip = gpiod_chip_open_by_name(chip_name.c_str());
    if (chip == nullptr) {
        close();
        return false;
    }

    if (!request_output(chip, line_num, "buzzer", 0, buzzer)) {
        close();
        return false;
    }
    return true;
}
void BuzzerDriver::close() {
    if (buzzer != nullptr) {
        gpiod_line_release(buzzer);
        buzzer = nullptr;
    }
    if (chip != nullptr) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
}
void BuzzerDriver::onTimer() {
    bool on = (active_code >> (31 - index)) & 1;

    index++;
    if (index > 31) {
        index = 0;
        if (repeat_count > 0){
            repeat_count--;
        }
        if (repeat_count == 0){
            active_code = 0;
        }
    }

        if (buzzer == nullptr) {
        return;
    }
    const int value = on ? 1 : 0;
    gpiod_line_set_value(buzzer, value);

}
} // namespace cubesat_io_node