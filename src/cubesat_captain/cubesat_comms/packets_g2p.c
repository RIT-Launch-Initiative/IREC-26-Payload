#include "cubesat_comms/packets_g2p.h"
#include "string.h"


int pack_arm_target(const struct ArmTarget *target, uint8_t *buf)
{
    buf[0] = target->shoulder_yaw;
    buf[1] = target->shoulder_pitch;
    buf[2] = target->elbow_pitch;
    buf[3] = target->wrist_pitch;
    return 4;
}
enum UnpackResult unpack_arm_target(const uint8_t *buf, uint32_t len, struct ArmTarget *target)
{
    if (len < 4) {
        return UnpackResult_TooShort;
    }

    target->shoulder_yaw = buf[0];
    target->shoulder_pitch = buf[1];
    target->elbow_pitch = buf[2];
    target->wrist_pitch = buf[3];

    return UnpackResult_AllGood;
}





static const uint32_t take_picture_location = 7;
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
  case Command_StopVideo:
    return 1;
  case Command_Callsign: 
    pack_callsign(&cmd_and_data->callsign, buf+1);
    return 1 + SIZEOF_PACKED_CALLSIGN;
  case Command_StartVideo:
    buf[1] = cmd_and_data->start_video.seconds;
    return 2;
  case Command_ReCrop:
    buf[1] = cmd_and_data->recrop.original_image;
    pack_phototransform(&cmd_and_data->recrop.transform, buf+1);
    return 1+ SIZEOF_PACKED_RECROP_DATA;

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
    buf[1] = cmd_and_data->shell_exec_return_info_request.exec_id;
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
  case Command_StopVideo:
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
      cmd_and_data->shell_exec_return_info_request.exec_id = buf[1];
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



int pack_int16(int16_t i, uint8_t *buf)
{
    memcpy(buf, &i, sizeof(int16_t));
    return sizeof(int16_t);
}
enum UnpackResult unpack_int16(const uint8_t *buf, uint32_t len, int16_t *i)
{
    if (len < 2) {
        return UnpackResult_TooShort;
    }

    memcpy(i, buf, sizeof(int16_t));

    return UnpackResult_AllGood;
}

int pack_uint16(uint16_t i, uint8_t *buf)
{
    memcpy(buf, &i, sizeof(uint16_t));
    return sizeof(uint16_t);
}
enum UnpackResult unpack_uint16(const uint8_t *buf, uint32_t len, uint16_t *i)
{
    if (len < 2) {
        return UnpackResult_TooShort;
    }

    memcpy(i, buf, sizeof(uint16_t));

    return UnpackResult_AllGood;
}

int pack_float(float f, uint8_t *buf)
{
    memcpy(buf, &f, sizeof(float));
    return sizeof(float);
}
enum UnpackResult unpack_float(const uint8_t *buf, uint32_t len, float *f)
{
    if (len < 4) {
        return UnpackResult_TooShort;
    }

    memcpy(f, buf, sizeof(float));

    return UnpackResult_AllGood;
}


const char *telemetry_type_to_str(enum TelemetryType typ) {
    switch (typ) {
    case TelemetryType_LandedHeartbeat:
        return "General";
    case TelemetryType_Actuators:
        return "Actuators";
    case TelemetryType_GNSS:
        return "GNSS";
    case TelemetryType_System:
        return "System";
    case TelemetryType_Orientations:
        return "Orientations";
    case TelemetryType_Temps:
        return "Temp";
    case TelemetryType_Power:
        return "Power";
    default:
        return "Unknown";
    }
}


void pack_callsign(const struct Callsign *callsign, uint8_t *buf){
  memcpy(buf, callsign->buf, 6);
}


int pack_phototransform(const struct PhotoTransform *tform, uint8_t *buf){
  uint32_t size = 0;
  pack_uint16(tform->left, buf);
  pack_uint16(tform->right, buf + size);
  pack_uint16(tform->top, buf + size);
  pack_uint16(tform->bottom, buf + size * 3);
  
  uint16_t packed_q_and_width = ((tform->quality & 0b111) << 13) | (tform->output_width & 0b11111111111);
  pack_uint16(packed_q_and_width, buf + size);
  return +size;
}

