#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

enum G2PPacketType {
    G2PPacketType_LinkTest = 0b00,
    G2PPacketType_LinkControl = 0b01,
    G2PPacketType_Command = 0b10,
    G2PPacketType_ImageControl = 0b11,
};

struct G2PLinkHeader {
    enum G2PPacketType packet_type;           // 2 bit
    uint8_t expected_packets_before_response; // 6 bit [0,MAX_PACKETS_BEFORE_RESPONSE] counts down
                                              // and then the last may have
                                              // needs_ack set
};

enum UnpackResult unpack_g2p_link_header(const uint8_t *buf, uint32_t len, struct G2PLinkHeader *header);
int pack_g2p_link_header(const struct G2PLinkHeader *header, uint8_t *buf);

struct StartVideoData {
    uint8_t seconds;
};

struct RecropData {
    uint8_t original_image;
    struct PhotoTransform transform;
};
#define SIZEOF_PACKED_RECROP_DATA (1 + SIZEOF_PACKED_PHOTOTRANSFORM)

struct ServoMotion
{
    uint8_t openness;
    uint8_t open_travel_time;
    uint8_t open_time;
    uint8_t close_travel_time;
    uint8_t closeness;
    uint8_t which_servo;
};
#define SIZEOF_PACKED_SERVO_MOTION 6

int pack_servo_motion(const struct ServoMotion *motion, uint8_t *buf);
enum UnpackResult unpack_servo_motion(const uint8_t *buf, int len, struct ServoMotion *motion);

struct MotorJog
{
    uint8_t motor_id;
    uint8_t duration_ticks;
    int16_t millivolts;
};
#define SIZEOF_PACKERD_MOTOR_JOG 4

int pack_motor_jog(const struct MotorJog *jog, uint8_t *buf);
enum UnpackResult unpack_motor_jog(const uint8_t *buf, int len, struct MotorJog *motion);

struct TelemetryRequestData
{
    enum TelemetryType telem_type;
};

struct Callsign {
    uint8_t buf[6];
};
#define SIZEOF_PACKED_CALLSIGN 6
void pack_callsign(const struct Callsign *callsign, uint8_t *buf);

#define MAX_BLOCKS_PER_REQUEST 60
struct ImageBlockRequest
{
    uint8_t image_id;
    uint8_t num; // max MAX_BLOCKS_PER_REQUEST
    uint16_t block_ids[MAX_BLOCKS_PER_REQUEST];
};
int pack_image_block_request(const struct ImageBlockRequest *req, uint8_t *buf);
enum UnpackResult unpack_image_block_request(uint8_t *buf,
                                             uint32_t len,
                                             struct ImageBlockRequest *req);

struct CommandAndData {
    enum Command command;
    union {
        // ForceManual
        // ForceFlight
        // ExpectFlight
        // UnexpectFlight
        struct StartVideoData start_video;
        // StopVideo
        struct PhotoTransform take_picture;
        uint8_t metadata_ask_image_id;
        struct RecropData recrop;

        struct ArmTarget send_arm_to_target;
        struct ArmTarget send_arm_to_target_and_come_back;
        struct ArmTarget send_arm_to_target_for_photo_and_come_back;
        struct ArmTarget send_idle_position;
        struct ArmTarget set_shoulder_position;

        struct MotorJog motor_jog;
        struct ServoMotion servo_motion;

        // ClearFlight
        struct TelemetryRequestData telem_request;

        struct Callsign callsign;
    };
};

// buf must be at least 255 bytes
// return the amount of the buffer filled
int pack_command_and_data(const struct CommandAndData *cmd_and_data, uint8_t *buf);

// returns true if valid, false if invalid
enum UnpackResult unpack_command_and_data(const uint8_t *buf, int len, struct CommandAndData *cmd_and_data);

#ifdef __cplusplus
}
#endif
