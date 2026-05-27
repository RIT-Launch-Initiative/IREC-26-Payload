#pragma once
#include <cstdint>

namespace StmBridge {

// TYPES ==================
enum class State {
    Chilling = 0,
    ArmMoving = 1,
    Servo1Moving = 2,
    Servo2Moving = 3,
    Servo3Moving = 4,
};

using StatusWord = uint16_t;

enum StatusBit {
    StatusBit_Booted = 0,            // set to 1 if board is ready
    StatusBit_State0 = 1,        // Arm
    StatusBit_State1 = 2, // Flipping Servo 1
    StatusBit_State2 = 3, // Flipping Servo 2
    
    StatusBit_MovingArmFailed = 5,  // Arm failed bc OCP
    StatusBit_WristServoEn = 6,     // Efuse enable
    StatusBit_FlipServoEn = 7,      // 8.4V Buck enable
    StatusBit_MotorEn = 8,          // not sleeping
    StatusBit_Overtemp = 9,

    // identify what kind of response this is 0 - 31
    StatusBit_RType0 = 11,
    StatusBit_RType1 = 12,
    StatusBit_RType2 = 13,
    StatusBit_RType3 = 14,
    StatusBit_RType4 = 15,
};

bool CheckStatusBit(StatusWord word, StatusBit bit_index);

struct ArmPose {
    int8_t shoulder_yaw;
    int8_t shoulder_pitch;
    int8_t elbow_pitch;
    int8_t wrist_pitch;
};

struct Status {
    StatusWord status_word;
    ArmPose pose;
    uint16_t uptime_lsw;
};

enum FlipServo {
    Servo1 = 0,
    Servo2 = 1,
    Servo3 = 2,
};

struct FlipServoMotion {
    uint8_t open_duration;         // How long to stay open for in 10 ms increments
    uint8_t openness;              // How much to open
    uint8_t open_travel_duration;  // How long to move the servo on open in 10 ms
                                   // increments
    uint8_t closedness;            // where to go after open duration
    uint8_t close_travel_duration; // How long to move the servo on close in 10 ms
                                   // increments

    // total duration in 10 ms increments
    uint32_t total_duration() const;
};

struct FlipServoMotionState {
    uint32_t iteration_started;
    FlipServoMotion motion;
};

struct Temperatures {
    int16_t link1_temp;
    int16_t link2_temp;
    int16_t stm_temp;
};

struct Vec3_16 {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct ServoTargets {
    uint16_t servo1;
    uint16_t servo2;
    uint16_t servo3;
};

} // namespace StmBridge