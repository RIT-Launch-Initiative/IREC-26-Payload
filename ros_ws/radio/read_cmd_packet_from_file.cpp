#include "packets.h"
#include <fstream>
#include <istream>
#include <ostream>
#include <stdio.h>
#include <string>
#include <vector>

void print_arm_target(const struct ArmTarget *target) {
  printf("sy = %d, sp = %d, ep = %d, wp = %d", target->shoulder_yaw,
         target->shoulder_pitch, target->elbow_pitch, target->wrist_pitch);
}

void print_cmd(const struct CommandAndData *cmd) {
  switch (cmd->command) {
  case Command_ForceManual:
    printf("Force Manual Control\n");
    break;
  case Command_ForceFlight:
    printf("Force Flight Start\n");
    break;
  case Command_ExpectFlight:
    printf("Expect Flight\n");
    break;
  case Command_UnexpectFlight:
    printf("Unexpect Flight\n");
    break;
  case Command_StartVideo:
    printf("Start Video: %d seconds\n", cmd->start_video.seconds);
    break;
  case Command_StopVideo:
    printf("Stop Video\n");
    break;
  case Command_SendArmTargetAndComeBack:
    printf("Send arm to target and back: ");
    print_arm_target(&cmd->send_arm_to_target_and_come_back);
    printf("\n");
    break;
  case Command_SendArmTargetForPhotoAndComeBack:
    printf("Send arm to target and back w/ photo: ");
    print_arm_target(&cmd->send_arm_to_target_and_come_back);
    printf("\n");
    break;
  case Command_SendIdlePosition:
    printf("Send idle position: ");
    print_arm_target(&cmd->send_arm_to_target_and_come_back);
    printf("\n");
    break;
  case Command_WriteArmSequence:
    printf(
        "Read Arm Sequence: path = %d, index = %d, delay = %d, take_pic = %s, ",
        cmd->write_arm_sequence.path_id, cmd->write_arm_sequence.index,
        cmd->write_arm_sequence.delay_before_next,
        cmd->write_arm_sequence.take_picture ? "true" : "false");
    print_arm_target(&cmd->write_arm_sequence.target);
    printf("\n");
    break;
  case Command_ReadArmSequence:
    printf("Read Arm Sequence: path = %d, index = %d, ",
           cmd->read_arm_sequence.path_id, cmd->read_arm_sequence.index);
    break;
  case Command_ExecuteArmSequence:
    printf("Execute Arm Sequence #%d\n", cmd->execute_arm_sequence.path_id);
    break;
  case Command_CancelExecutingArmSequence:
    printf("Cancel Arm Sequence\n");
    break;
  case Command_ZeroShoulder_AssumeOpen:
    printf("Zero Shoulder (assume open)\n");
    break;
  case Command_RunOpenSequence:
    printf("Run Open Sequence\n");
    break;
  case Command_ShellExec:
    printf("Execute shell command: %.*s\n", cmd->shell_exec.length,
           cmd->shell_exec.buf);
    break;
  case Command_ShellExecInfo:
    printf("Get Info of shell exec #%d\n",
           cmd->shell_exec_return_info_request.command_id);
    break;
  case Command_ShellReadStdout:
    printf("Read stdout: index=%d, compressed=%s\n",
           cmd->shell_read_stdout.index,
           cmd->shell_read_stdout.get_compressed ? "true" : "false");
    break;
  case Command_ShellReadStderr:
    printf("Read stderr: index=%d, compressed=%s\n",
           cmd->shell_read_stdout.index,
           cmd->shell_read_stdout.get_compressed ? "true" : "false");
    break;
  case Command_ClearFlightDANGER:
    printf("Clear Flight\n");
    break;
  case Command_GetFlightNumber:
    printf("Get Flight Number\n");
    break;
  case Command_TelemetryRequest:
    printf("Telemetry Request: %s\n",
           telemetry_type_to_str(cmd->telem_request.telem_type));
    break;
  default:
    printf("Unknown Command\n");
    break;
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("No enough args: ./exe path/to/packet/directory");
    return 1;
  }
  std::string dir_path = argv[1];
  std::string packet_path = dir_path + "/packet";

  std::ifstream infile(packet_path, std::ios_base::binary);
  std::vector<uint8_t> buffer{std::istreambuf_iterator<char>(infile),
                              std::istreambuf_iterator<char>()};
  printf("File length: %d\n", (int)buffer.size());

  struct CommandAndData cmd = {};
  enum UnpackResult res =
      unpack_command_and_data(buffer.data(), buffer.size(), &cmd);
  switch (res) {
  case UnpackResult_AllGood:
    printf("Success\n");
    break;
  case UnpackResult_NoCommand:
    printf("Failed to read: No Command specifier in packet\n");
    return -1;
  case UnpackResult_TooLong:
    printf("Failed to read: too long for read bytes\n");
    return -1;
  case UnpackResult_TooShort:
    printf("Failed to read: Not enough bytes for packet type: %d\n",
           cmd.command);
    return -1;
  case UnpackResult_Unimplemented:
    printf("Failed to read: Unimplemented\n");
    return -1;
  case UnpackResult_UnknownCommand:
    printf("Failed to read: Unknown Command Specifier\n");
    return -1;
  }
  return 0;
}