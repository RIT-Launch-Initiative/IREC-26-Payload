#include "lora.h"
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
  bool needs_ack;                           // 1 bit
  uint8_t expected_packets_before_response; // 5 bit [0,31] counts down
                                            // and then the last may have
                                            // needs_ack set
};

// From GS to Payload
enum Command {
  Command_ForceManual,  // stop anything on a timer and listen for radio message
  Command_ForceFlight,  // in case boost detect failed
  Command_ExpectFlight, // start going faster and sending me more packets for
                        // status of flight bc you're about to take off
  Command_UnexpectFlight, // go back to normal waitin on the pad mode
  Command_StartVideo,
  Command_SendArmTargetAndComeBack, // go to this position and go back to rest
                                    // position after. Don't take a picture
  Command_SendArmTargetForPhotoAndComeBack, // go to this position, take a
                                            // picture, return to transmit
                                            // position
  Command_SendIdlePosition,           // treat the sent position as the 'home'
                                      // position for sending UNLESS you don't
                                      // hear back
  Command_WriteArmSequence,           // add positions to manual arm sequence,
  Command_ReadArmSequence,            // read positions to manual arm sequence,
  Command_ExecuteArmSequence,         //  execute manual arm sequence,
  Command_CancelExecutingArmSequence, //  cancel current executing manual arm
                                      //  sequence,
  Command_ZeroShoulder_AssumeOpen,    // points joints straight up and spins the
                                      // shoulder joint to try to hit the hal
                                      // effect sensor
  Command_RunOpenSequence,            // if servos: Run them to flip us upright
                           // if arm: return to 'stowed' position if not close
                           // then do the sequence to flip us out and up
  Command_ManualServoPositions,

  Command_ShellExec, // execute a shell command - does not necessarily wait for
                     // it to finish (tho like don't run an infinitely running
                     // command unless you mean to)
  Command_ShellExecInfo, // get results of previous command, response of exit
                         // code and length of stdout, stderr
  Command_ShellReadStdout,
  Command_ShellReadStderr,
  Command_ClearFlightDANGER, // DANGER delete the 'we launched' flag and reset
                             // counters and delete directories for images and
                             // shell commands. Restarts all programs, may blink
                             // offline for a sec
  Command_GetFlightNumber, // returns the identifier for the current flight so that the gs can know and start a new directory accordingly
  Command_TelemetryRequest,  // Get really detailed info about a subsystem, can
                             // be 'responded' to w/o being requested
};


struct StartVideoData {
  uint8_t seconds;
};

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
struct ShellExecData {
  uint8_t length;
  uint8_t buf[128]; // command to run
};

struct ShellReadOutputRequest{
    bool get_compressed; // if set, read from 
    uint16_t index; // index fo 128 byte chunk to send. 
};

struct TelemetryRequestData {
  enum TelemetryType telem_type;
};


struct ShellExecReturnData {
  uint16_t stdout_len;
  uint16_t stderr_len;
  uint16_t stdout_compressed_len;
  uint16_t stderr_compressed_len;
  int return_code;
};


struct CommandAndData {
  enum Command command;
  union {
    // ForceManual
    // ForceFlight
    // ExpectFlight
    // UnexpectFlight
    struct StartVideoData start_video;
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
    struct ShellExecReturnData shell_exec_return;
    struct ShellReadOutputRequest shell_read_stdout;
    struct ShellReadOutputRequest shell_read_stderr;
    // ClearFlight
    struct TelemetryRequestData telem_request;
  };
};
