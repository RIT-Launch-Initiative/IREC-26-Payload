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


struct StartVideoData {
  uint8_t seconds;
};

struct RecropData
{
    uint8_t original_image;
    struct PhotoTransform transform;
};
#define SIZEOF_PACKED_RECROP_DATA (1 + SIZEOF_PACKED_PHOTOTRANSFORM)

#define MAX_DELAY_IN_SEQUENCE 127
struct WriteArmSequenceData {
  uint8_t path_id;
  uint8_t index;
  bool take_picture;
  uint8_t delay_before_next; // [0,127] seconds
  struct ArmTarget target;
};

struct ReadArmSequenceData {
  uint8_t path_id;
  uint8_t index;
};
struct ExecuteArmSequenceData {
  uint8_t path_id;
};

// Maybe
enum ServoOrder {
  ServoOrder_123 = 0,
  ServoOrder_132 = 1,
  ServoOrder_213 = 2,
  ServoOrder_231 = 3,
  ServoOrder_312 = 4,
  ServoOrder_321 = 5,
};

// Maybe
struct ManualServoPositionsData {
  uint8_t servo1;
  uint8_t servo2;
  uint8_t servo3;
  bool return_to_home_after;           // 1 bit
  bool hold_position_across_movements; // 1 bit
  enum ServoOrder order;               // 3 bit
};

#define MAX_SHELL_EXEC_LEN 128
struct ShellExecData {
    // exec-id determined by next available slot on payload
    uint8_t length;
    uint8_t buf[MAX_SHELL_EXEC_LEN]; // command to run
};

struct ShellReadOutputRequest {
    uint8_t exec_id; // id of execution to read data from
    uint16_t index;  // index of SHELL_OUTPUT_CHUNK_SIZE byte sized chunk to send.
    uint8_t length; // length in # of chunks to get at once (MSBit used to say compressed or uncompressed. Maximum of MAX_PACKETS_BEFORE_RESPONSE
    bool get_compressed; // if set, read from compressed version (packed into length field)
};
#define SIZEOF_PACKED_SHELL_READ_OUTPUT_REQ 4

struct TelemetryRequestData {
  enum TelemetryType telem_type;
};

struct ShellExecReturnDataRequest {
    uint8_t exec_id;
};

struct Callsign
{
    uint8_t buf[6];
};
#define SIZEOF_PACKED_CALLSIGN 6
void pack_callsign(const struct Callsign *callsign, uint8_t *buf);

struct CommandAndData
{
    enum Command command;
    union {
        // ForceManual
        // ForceFlight
        // ExpectFlight
        // UnexpectFlight
        struct StartVideoData start_video;
        // StopVideo
        struct RecropData recrop;

        struct ArmTarget send_arm_to_target_and_come_back;
        struct ArmTarget send_arm_to_target_for_photo_and_come_back;
        struct ArmTarget send_idle_position;
        struct WriteArmSequenceData write_arm_sequence;
        struct ReadArmSequenceData read_arm_sequence;
        struct ExecuteArmSequenceData execute_arm_sequence;
        // CancelExecutingArmSequence
        // ZeroShoulder Assume Open
        // RunOpenSequence
        struct ManualServoPositionsData manual_zero_positions;

        struct ShellExecData shell_exec;
        struct ShellExecReturnDataRequest shell_exec_return_info_request;
        struct ShellReadOutputRequest shell_read_stdout;
        struct ShellReadOutputRequest shell_read_stderr;
        // ClearFlight
        struct TelemetryRequestData telem_request;

        struct Callsign callsign;
    };
};

// buf must be at least 255 bytes
// return the amount of the buffer filled
int pack_command_and_data(const struct CommandAndData *cmd_and_data, uint8_t *buf);

// returns true if valid, false if invalid
enum UnpackResult unpack_command_and_data(const uint8_t *buf,
                                          int len,
                                          struct CommandAndData *cmd_and_data);

#ifdef __cplusplus
}
#endif
