cmake_minimum_required(VERSION 3.16)


add_library(cubesat_comms STATIC lora.c packets_g2p.c packets_p2g.c)
target_include_directories(cubesat_comms PUBLIC include/)