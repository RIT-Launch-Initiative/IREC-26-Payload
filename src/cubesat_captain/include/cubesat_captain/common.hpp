#pragma once
#include "cubesat_msgs/msg/accel_sample.h"
#include <cmath>

double accel_magnitude(const cubesat_msgs::msg::AccelSample &sample) ;
cubesat_msgs::msg::AccelSample normalize_accel(const cubesat_msgs::msg::AccelSample &sample) ;