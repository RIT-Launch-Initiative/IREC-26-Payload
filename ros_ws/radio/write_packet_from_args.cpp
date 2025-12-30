#include "packets.h"
#include <fstream>
#include <istream>
#include <ostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

typedef int (*parser_func)(int argc, char **argv, uint8_t *output_buf,
                           enum Command cmd);

struct parser_pair {
  parser_func f;
  enum Command data;
};

int no_arg_parser(int argc, char **argv, uint8_t *output_buf,
                  enum Command cmd) {
  struct CommandAndData cd = {.command = cmd};
  return pack_command_and_data(&cd, output_buf);
}

int start_video_parser(int argc, char **argv, uint8_t *output_buf,
                       enum Command _c) {
  if (argc < 1) {
    fprintf(stderr,
            "StartVideo requires one argument (seconds to record for)\n");
    return -1;
  }
  int a = atoi(argv[0]);
  if (a == 0) {
    fprintf(stderr, "Invalid length\n");
    return -1;
  }
  if (a > 255) {
    fprintf(stderr, "length too long (max 255)\n");
  }
  struct CommandAndData cd = {.command = Command_StartVideo,
                              .start_video = {.seconds = (uint8_t)a}};
  return pack_command_and_data(&cd, output_buf);
}
struct string_pair {
  const char *name;
  enum Command cmd;
};

string_pair strings[Command_MaxCommand] = {
    {"ForceManual", Command_ForceManual},
    {"ForceFlight", Command_ForceFlight},
    {"ExpectFlight", Command_ExpectFlight},
    {"UnexpectFlight", Command_UnexpectFlight},
    {"StartVideo", Command_StartVideo},
    {"StopVideo", Command_StopVideo},
    {"SendArmTargetAndComeBack", Command_SendArmTargetAndComeBack},
    {"SendArmTargetForPhotoAndComeBack",
     Command_SendArmTargetForPhotoAndComeBack},
    {"SendIdlePosition", Command_SendIdlePosition},
    {"WriteArmSequence", Command_WriteArmSequence},
    {"ReadArmSequence", Command_ReadArmSequence},
    {"ExecuteArmSequence", Command_ExecuteArmSequence},
    {"CancelExecutingArmSequence", Command_CancelExecutingArmSequence},
    {"ZeroShoulder_AssumeOpen", Command_ZeroShoulder_AssumeOpen},
    {"RunOpenSequence", Command_RunOpenSequence},
    {"ShellExec", Command_ShellExec},
    {"ShellExecInfo", Command_ShellExecInfo},
    {"ShellReadStdout", Command_ShellReadStdout},
    {"ShellReadStderr", Command_ShellReadStderr},
    {"ClearFlightDANGER", Command_ClearFlightDANGER},
    {"GetFlightNumber", Command_GetFlightNumber},
    {"TelemetryRequest", Command_TelemetryRequest},

};

enum Command str_to_cmd(const char *str) {
  std::string sstr{str};

  for (size_t i = 0; i < Command_MaxCommand; i++) {
    if (sstr == strings[i].name) {
      return strings[i].cmd;
    }
  }

  return Command_MaxCommand;
}

int main(int argc, char *argv[]) {

  parser_pair parsers[Command_MaxCommand] = {};
  parsers[Command_ForceFlight] = {no_arg_parser, Command_ForceFlight};
  parsers[Command_ExpectFlight] = {no_arg_parser, Command_ExpectFlight};
  parsers[Command_UnexpectFlight] = {no_arg_parser, Command_UnexpectFlight};
  parsers[Command_CancelExecutingArmSequence] = {
      no_arg_parser, Command_CancelExecutingArmSequence};
  parsers[Command_ZeroShoulder_AssumeOpen] = {no_arg_parser,
                                              Command_ZeroShoulder_AssumeOpen};

  parsers[Command_ClearFlightDANGER] = {no_arg_parser,
                                        Command_ClearFlightDANGER};
  parsers[Command_StartVideo] = {start_video_parser, Command_StartVideo};

  parsers[Command_StopVideo] = {no_arg_parser, Command_StopVideo};

  if (argc < 3) {
    fprintf(stderr,
            "No enough args: ./exe path/to/packet/file command_specifier "
            "[args]");
    return EXIT_FAILURE;
  }
  enum Command cmd = str_to_cmd(argv[2]);
  if (cmd >= Command_MaxCommand) {
    fprintf(stderr, "Invalid packet type\n");
    return EXIT_FAILURE;
  }
  parser_func f = parsers[cmd].f;
  enum Command dat = parsers[cmd].data;
  if (f == NULL) {
    fprintf(stderr, "No handler for that packet type yet\n");
    return EXIT_FAILURE;
  }
  uint8_t buf[255] = {0};

  int ret = f(argc - 3, &argv[3], buf, dat);
  if (ret < 1) {
    // error writing
    return ret;
  }
  int len = ret;

  FILE *fil = fopen(argv[1], "w");
  if (fil == NULL) {
    fprintf(stderr, "Failed to open file\n");
    return EXIT_FAILURE;
  }
  size_t fret = fwrite((void *)&buf[0], sizeof(uint8_t), len, fil);
  if (fret != len) {
    fprintf(stderr, "failed to write file\n");
    return EXIT_FAILURE;
  }

  int close_ret = fclose(fil);
  if (close_ret != 0) {
    fprintf(stderr, "error closing: %d\n", close_ret);
    return close_ret;
  }

  return 0;
}