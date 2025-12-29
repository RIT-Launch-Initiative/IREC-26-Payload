#pragma once
#include "common.h"
#include "lora.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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
  // Command_ManualServoPositions,

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
  Command_GetFlightNumber,   // returns the identifier for the current flight so
                             // that the gs can know and start a new directory
                             // accordingly
  Command_TelemetryRequest,  // Get really detailed info about a subsystem, can
                             // be 'responded' to w/o being requested
  Command_MaxCommand,
};

struct StartVideoData {
  uint8_t seconds;
};

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
  uint8_t length;
  uint8_t buf[MAX_SHELL_EXEC_LEN]; // command to run
};

struct ShellReadOutputRequest {
  bool get_compressed; // if set, read from
  uint16_t index;      // index fo 128 byte chunk to send.
};

struct TelemetryRequestData {
  enum TelemetryType telem_type;
};

struct ShellExecReturnDataRequest {
  uint8_t command_id;
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
    struct ShellExecReturnDataRequest shell_exec_return_info_request;
    struct ShellReadOutputRequest shell_read_stdout;
    struct ShellReadOutputRequest shell_read_stderr;
    // ClearFlight
    struct TelemetryRequestData telem_request;
  };
};

// buf must be atleast sizeof(ArmTarget)
int pack_arm_target(const struct ArmTarget *target, uint8_t *buf) {
  buf[0] = target->shoulder_yaw;
  buf[1] = target->shoulder_pitch;
  buf[2] = target->elbow_pitch;
  buf[3] = target->wrist_pitch;
  return 4;
}

static const size_t take_picture_location = 7;
static const uint8_t take_picture_mask = 1 << take_picture_location;
static const uint8_t delay_mask = (uint8_t)~take_picture_mask;

int pack_write_arm_sequence_data(const struct WriteArmSequenceData *data,
                                 uint8_t *buf) {
  buf[0] = data->path_id;
  buf[1] = data->index;

  buf[2] = data->delay_before_next;
  if (buf[2] > MAX_DELAY_IN_SEQUENCE) {
    buf[2] = MAX_DELAY_IN_SEQUENCE;
  }
  if (data->take_picture) {
    buf[2] |= take_picture_mask;
  }
  return 3 + pack_arm_target(&data->target, &buf[3]);
}

enum UnpackResult unpack_arm_target(const uint8_t *buf, int len,
                                    struct ArmTarget *target) {
  if (len < 4) {
    return UnpackResult_TooShort;
  }
  target->shoulder_yaw = buf[0];
  target->shoulder_pitch = buf[1];
  target->elbow_pitch = buf[2];
  target->wrist_pitch = buf[3];
  return UnpackResult_AllGood;
}

enum UnpackResult
unpack_write_arm_sequence_data(const uint8_t *buf, int len,
                               struct WriteArmSequenceData *data) {
  if (len < 3) {
    return UnpackResult_TooShort;
  }

  data->path_id = buf[0];
  data->index = buf[1];

  data->take_picture = (buf[2] & take_picture_mask) != 0;
  data->delay_before_next = buf[2] & 0b1111111;

  return unpack_arm_target(&buf[3], len - 3, &data->target);
}

int pack_shell_exec(const struct ShellExecData *exec, uint8_t *buf) {
  buf[0] = exec->length;
  if (buf[0] > MAX_SHELL_EXEC_LEN) {
    buf[0] = MAX_SHELL_EXEC_LEN;
  }
  memcpy(&buf[1], exec->buf, buf[0]);
  return 1 + buf[0];
}

enum UnpackResult unpack_shell_exec(const uint8_t *buf, int len,
                                    struct ShellExecData *exec) {
  if (len < 1) {
    return UnpackResult_TooShort;
  }
  uint8_t buf_len = buf[0];
  if (len - 1 < buf_len) {
    return UnpackResult_TooShort;
  }
  if (buf_len > MAX_SHELL_EXEC_LEN) {
    return UnpackResult_TooLong;
  }
  memcpy(exec->buf, &buf[1], buf_len);
  return UnpackResult_AllGood;
}

int pack_shell_read_output_request(const struct ShellReadOutputRequest *request,
                                   uint8_t *buf) {
  static const uint16_t want_compressed_mask = 1 << 15;
  static const uint16_t index_mask = (uint16_t)~want_compressed_mask;

  uint16_t packet = (request->get_compressed ? want_compressed_mask : 0) |
                    (request->index & index_mask);
  buf[0] = (packet >> 8) & 0xff;
  buf[1] = (packet) & 0xff;
  return 2;
}

enum UnpackResult
unpack_shell_read_output_request(const uint8_t *buf, int len,
                                 struct ShellReadOutputRequest *request) {
  static const uint16_t want_compressed_mask = 1 << 15;
  static const uint16_t index_mask = (uint16_t)~want_compressed_mask;
  if (len < 2) {
    return UnpackResult_TooShort;
  }

  uint16_t packet = (buf[0] << 8) | buf[1];
  request->get_compressed = (packet & want_compressed_mask) >> 15;
  request->index = packet & index_mask;

  return UnpackResult_AllGood;
}

// buf must be at least 255 bytes
// return the amount of the buffer filled
int pack_command_and_data(const struct CommandAndData *cmd_and_data,
                          uint8_t *buf) {
  buf[0] = (uint8_t)cmd_and_data->command;
  switch (cmd_and_data->command) {
  case Command_ForceManual:
  case Command_ForceFlight:
  case Command_ExpectFlight:
  case Command_UnexpectFlight:
  case Command_ClearFlightDANGER:
  case Command_CancelExecutingArmSequence:
  case Command_ZeroShoulder_AssumeOpen:
  case Command_RunOpenSequence:
  case Command_GetFlightNumber:
    return 1;
  case Command_StartVideo:
    buf[1] = cmd_and_data->start_video.seconds;
    return 2;
  case Command_SendArmTargetAndComeBack:
    return 1 + pack_arm_target(&cmd_and_data->send_arm_to_target_and_come_back,
                               &buf[1]);
  case Command_SendArmTargetForPhotoAndComeBack:
    return 1 + pack_arm_target(
                   &cmd_and_data->send_arm_to_target_for_photo_and_come_back,
                   &buf[1]);
  case Command_SendIdlePosition:
    return 1 + pack_arm_target(&cmd_and_data->send_idle_position, &buf[1]);
  case Command_WriteArmSequence:
    return 1 + pack_write_arm_sequence_data(&cmd_and_data->write_arm_sequence,
                                            &buf[1]);
  case Command_ReadArmSequence:
    buf[1] = cmd_and_data->read_arm_sequence.path_id;
    buf[2] = cmd_and_data->read_arm_sequence.index;
    return 3;
  case Command_ExecuteArmSequence:
    buf[1] = cmd_and_data->execute_arm_sequence.path_id;
    return 2;
  case Command_ShellExec:
    return 1 + pack_shell_exec(&cmd_and_data->shell_exec, buf);
  case Command_ShellExecInfo:
    buf[1] = cmd_and_data->shell_exec_return_info_request.command_id;
    return 2;
  case Command_ShellReadStdout:
    return 1 + pack_shell_read_output_request(&cmd_and_data->shell_read_stdout,
                                              &buf[1]);
  case Command_ShellReadStderr:
    return 1 + pack_shell_read_output_request(&cmd_and_data->shell_read_stderr,
                                              &buf[1]);
  case Command_TelemetryRequest:
    buf[1] = cmd_and_data->telem_request.telem_type;
    return 2;
  case Command_MaxCommand:
    // WARNING - invalid command: unpack will tell them but this is weird
    return 1;
  }
  return 1;
}

// returns true if valid, false if invalid
enum UnpackResult unpack_command_and_data(const uint8_t *buf, int len,
                                          struct CommandAndData *cmd_and_data) {
  if (len < 1) {
    return UnpackResult_NoCommand;
  }
  cmd_and_data->command = (enum Command)buf[0];
  if (cmd_and_data->command >= Command_MaxCommand) {
    return UnpackResult_UnknownCommand;
  }
  int data_len = len - 1;
  const uint8_t *data_buf = &buf[1];
  switch (cmd_and_data->command) {
  case Command_ForceManual:
  case Command_ForceFlight:
  case Command_ExpectFlight:
  case Command_UnexpectFlight:
  case Command_ClearFlightDANGER:
  case Command_CancelExecutingArmSequence:
  case Command_ZeroShoulder_AssumeOpen:
  case Command_RunOpenSequence:
  case Command_GetFlightNumber:
    return UnpackResult_AllGood; // no data with these messages
  case Command_StartVideo:
    if (data_len >= 1) {
      cmd_and_data->start_video.seconds = data_buf[0];
      return UnpackResult_AllGood;
    } else {
      return UnpackResult_TooShort;
    }
  case Command_SendArmTargetAndComeBack:
    return unpack_arm_target(data_buf, data_len,
                             &cmd_and_data->send_arm_to_target_and_come_back);
  case Command_SendArmTargetForPhotoAndComeBack:
    return unpack_arm_target(
        data_buf, data_len,
        &cmd_and_data->send_arm_to_target_for_photo_and_come_back);
  case Command_SendIdlePosition:
    return unpack_arm_target(data_buf, data_len,
                             &cmd_and_data->send_idle_position);
  case Command_WriteArmSequence:
    return unpack_write_arm_sequence_data(data_buf, data_len,
                                          &cmd_and_data->write_arm_sequence);
  case Command_ReadArmSequence:
    if (data_len >= 2) {
      cmd_and_data->read_arm_sequence.path_id = data_buf[0];
      cmd_and_data->read_arm_sequence.index = data_buf[1];
      return UnpackResult_AllGood;
    } else {
      return UnpackResult_TooShort;
    }
  case Command_ExecuteArmSequence:
    if (data_len >= 1) {
      cmd_and_data->execute_arm_sequence.path_id = data_buf[0];
    } else {
      return UnpackResult_TooShort;
    }

  case Command_ShellExec:
    return unpack_shell_exec(data_buf, data_len, &cmd_and_data->shell_exec);
  case Command_ShellExecInfo:
    if (data_len < 1) {
      return UnpackResult_TooShort;
    } else {
      cmd_and_data->shell_exec_return_info_request.command_id = buf[1];
      return UnpackResult_AllGood;
    }
  case Command_ShellReadStdout:
    return unpack_shell_read_output_request(data_buf, data_len,
                                            &cmd_and_data->shell_read_stdout);
  case Command_ShellReadStderr:
    return unpack_shell_read_output_request(data_buf, data_len,
                                            &cmd_and_data->shell_read_stderr);
  case Command_TelemetryRequest:
    if (data_len > 0) {
      return UnpackResult_AllGood;
    } else {
      return UnpackResult_TooShort;
    }
  case Command_MaxCommand:
    return UnpackResult_UnknownCommand;
  }
  return UnpackResult_UnknownCommand;
}