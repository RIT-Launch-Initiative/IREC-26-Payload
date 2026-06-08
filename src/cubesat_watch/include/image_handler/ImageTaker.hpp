#pragma once
#include <cstdint>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

struct Image {
    void *mmapped_memory;
    uint32_t length;
    cv::Mat mat;
};

Image &get_global_image();

bool take_global_image();
void free_global_image();