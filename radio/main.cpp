#include "packets.h"
#include <stdio.h>

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
    printf("Read Arm Sequence: path = %d, index = %d, delay = %d, take_pic = %s, ", cmd->write_arm_sequence.path_id, cmd->write_arm_sequence.index, cmd->write_arm_sequence.delay_before_next, cmd->write_arm_sequence.take_picture ? "true" : "false");
    print_arm_target(&cmd->write_arm_sequence.target);
    printf("\n");
    break;
  case Command_ReadArmSequence:
    printf("Read Arm Sequence: path = %d, index = %d, ", cmd->read_arm_sequence.path_id, cmd->read_arm_sequence.index);
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
    printf("Execute shell command: %.*s\n", cmd->shell_exec.length, cmd->shell_exec.buf);
    break;
  case Command_ShellExecInfo:
    printf("Get Info of shell exec #%d\n", cmd->shell_exec_return_info_request.command_id);
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

int main() {

  uint8_t buf[255] = {0};

  struct CommandAndData cmd = {
      .command = Command_WriteArmSequence,
      .write_arm_sequence =
          {
              .path_id = 1,
              .index = 0,
              .take_picture = true,
              .delay_before_next = 100,
              .target =
                  {
                      .shoulder_yaw = 90,
                      .shoulder_pitch = 12,
                      .elbow_pitch = 45,
                      .wrist_pitch = 100,
                  },
          },
  };
  int len = pack_command_and_data(&cmd, buf);
  struct CommandAndData cmd2 ;
  unpack_command_and_data(buf, len, &cmd2);
  printf("Len: %d\n", len);

  print_cmd(&cmd);
  print_cmd(&cmd2);

  return 0;
}