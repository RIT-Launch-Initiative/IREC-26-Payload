#pragma once
#include <cstdint>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


cv::Mat take_image_and_crop(std::string path, uint16_t cropLeft, uint16_t cropRight, uint16_t cropTop, uint16_t cropBottom,
                       uint16_t downscaleWidth);
