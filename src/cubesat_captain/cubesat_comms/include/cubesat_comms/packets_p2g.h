#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

enum P2GPacketType {
    P2GPacketType_LinkTestResponse = 0b00,
    P2GPacketType_LinkControl = 0b01,
    P2GPacketType_CommandResponse = 0b10,
    P2GPacketType_ImageResponse = 0b11,
};

struct P2GLinkHeader {
    enum P2GPacketType packet_type;           // 2 bit
    uint8_t expected_packets_before_response; // 6 bit [0,63] counts down
};

enum UnpackResult unpack_p2g_link_header(const uint8_t *buf, uint32_t len, struct P2GLinkHeader *header);
int pack_p2g_link_header(const struct P2GLinkHeader *header, uint8_t *buf);

struct ImageData {
    uint8_t buf[IMAGE_DATA_SIZE];
    // helpers defined by ssdv but we can unpack them to read
    uint8_t image_id;
    uint16_t block_index;
};

#define SIZEOF_PACKED_IMAGE_DATA IMAGE_DATA_SIZE
enum UnpackResult unpack_image_data(const uint8_t *buf, uint32_t len, struct ImageData *header);
// ignores image_id and block_index since those come from the buf
int pack_imagedata(const struct ImageData *data, uint8_t *buf);

struct ShellExecReturnData {
    uint8_t exec_id; // id of this shell execution
    uint16_t stdout_len;
    uint16_t stderr_len;
    uint16_t stdout_compressed_len;
    uint16_t stderr_compressed_len;
    int return_code;
};
#define SIZEOF_PACKED_SHELL_EXEC_RETURN_DATA (1 + 2 * 4 + 4)

struct ShellReadOutputData {
    uint8_t exec_id;  // id of this shell execution
    uint16_t index;   // maybe MSB means compressed version, 15 other bits specify which 128 byte chunk to use
    uint8_t buf[128]; // amount filled depends on std(out/err) and which chunk you
                      // are reading
};

enum FlightPhase {
    FlightPhase_Starting = 0,
    FlightPhase_Pad = 1,
    FlightPhase_ExpectingLaunch = 2,
    FlightPhase_Flight = 3,
    FlightPhase_LandedFlipping = 4,
    FlightPhase_Unfolding = 5,
    FlightPhase_AutoCamera = 6,
    FlightPhase_ManualControl = 7,
    FlightPhase_Emergency = 8,
};

enum StatusBit {

    StatusBit_Active = 0,
    StatusBit_Autonomous = 1,
    StatusBit_LastArmMoveStalled = 2,
    StatusBit_LastServoMoveStalled = 3,
    StatusBit_ArmMoving = 4,
    StatusBit_ServoMoving = 5,
    StatusBit_InIdlePosition = 6,
    StatusBit_MotorsOverTemp = 7,
    StatusBit_RadioOverTemp = 8,
    StatusBit_GPSHasFix = 9,
    StatusBit_RuncamOn = 10,
    StatusBit_CantTrustLink2Imu = 11,
    StatusBit_StmBooted = 12,

};

struct FlightState {
    enum FlightPhase phase;
    uint16_t status_bits;
};
#define SIZEOF_PACKED_FLIGHT_STATE 3

struct LandedHeartbeatStats {
    struct FlightState state;
    uint8_t next_image_id;
    uint8_t next_exec_id;
    struct ArmTarget arm_position;
    int16_t battery_mV;
    uint8_t motor_temp;
    uint8_t radio_temp;
};
#define SIZEOF_PACKED_LANDED_HEARTBEAT_STATS (SIZEOF_PACKED_FLIGHT_STATE + 1 + 1 + SIZEOF_PACKED_ARM_TARGET + 2 + 1 + 1)

struct FlightHeartbeatStats {
    struct FlightState state;
    float latitude;
    float longitude;
    uint16_t altitude;
    uint16_t s_since_boost;
    int16_t battery_mV;
    uint8_t radio_temp;
};
#define SIZEOF_PACKED_FLIGHT_HEARTBEAT_STATS (SIZEOF_PACKED_FLIGHT_STATE + 4 * 2 + 2 + 2 + 2 + 1)

/**
 * @brief Data about the state when the image was taken and parameters of its taking
 */
struct ImageMetadata {
    uint8_t image_id;
    uint32_t timestamp;
    uint16_t num_blocks;
    struct ArmTarget location;
    struct PhotoTransform transform;
    float latitude;
    float longitude;
};

#define SIZEOF_PACKED_IMAGE_METADATA (1 + 4 + 2 + SIZEOF_PACKED_ARM_TARGET + SIZEOF_PACKED_PHOTOTRANSFORM + 2 * 4)

int pack_image_metadata(const struct ImageMetadata *target, uint8_t *buf);
enum UnpackResult unpack_image_metadata(const uint8_t *buf, uint32_t len, struct ImageMetadata *target);

struct ActuatorPositions {
    struct ArmTarget arms;
    uint8_t servo1;
    uint8_t servo2;
    uint8_t servo3;
    uint8_t servo4;
};

int pack_actuator_positions(const struct ActuatorPositions *target, uint8_t *buf);
enum UnpackResult unpack_actuator_positions(const uint8_t *buf, uint32_t len, struct ActuatorPositions *target);

#define SIZEOF_PACKED_ACTUATOR_POSITIONS = SIZEOF_PACKED_ARM_TARGET + 4

struct IMUs {
    struct v3int16 base;
    struct v3int16 link1;
    struct v3int16 link2;
};

int pack_imus(const struct IMUs *target, uint8_t *buf);
enum UnpackResult unpack_imus(const uint8_t *buf, uint32_t len, struct IMUs *target);

struct Telemetry {
    enum TelemetryType telem_type;
    union {
        struct FlightHeartbeatStats flight_heartbeat_stats;
        struct LandedHeartbeatStats landed_heartbeat_stats;
        struct ActuatorPositions actuators;
        int gnss;
        struct IMUs orientations;
        int temps;
        int power;
    };
};
#define MAX_SIZEOF_PACKED_TELEMETRY                                                                                    \
    (1 + MAX7(SIZEOF_PACKED_HEARTBEAT_STATS, SIZEOF_PACKED_GENERAL_STATS_DETAILED, SIZEOF_PACKED_ACTUATOR_POSITIONS,   \
              4, 4, 4, 4))

struct CommandResponse {
    // send just command and nothing to indicate an acknowledge
    // if requires response, send cmd and corresponding data
    // if doesnt need an ack or data, can just listen and not explicitly reply
    // all entries can be sent unprompted as they can be fully understood without context.
    enum Command cmd;
    union {
        struct ShellExecReturnData shell_exec_info;
        struct ShellReadOutputData shell_stdout_data;
        struct ShellReadOutputData shell_stderr_data;
        struct ImageMetadata image_metadata;
        struct Telemetry telemetry;
    };
};

struct PacketP2G {
    struct P2GLinkHeader header;
    union {
        int link_test_data;
        int link_control_data;
        struct CommandResponse command_response;
        int image_data;
    };
};

int pack_command_response(const struct CommandResponse *cmd, uint8_t *buf);
enum UnpackResult unpack_command_response(const uint8_t *buf, uint32_t len, struct CommandResponse *resp);

int pack_telemetry(const struct Telemetry *telem, uint8_t *buf);
enum UnpackResult unpack_telemetry(const uint8_t *buf, uint32_t len, struct Telemetry *telem);

#ifdef __cplusplus
}
#endif
