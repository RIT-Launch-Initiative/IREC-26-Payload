#include "fuzztest/fuzztest.h"
#include "packets.h"
#include "gtest/gtest.h"
#include <vector>

void StartVideoCommandThereAndBack(uint8_t video_len) {
  struct CommandAndData cmd = {
      .command = Command_StartVideo,
      .start_video = {.seconds = video_len},
  };
  uint8_t buf[255] = {0};
  int len = pack_command_and_data(&cmd, buf);
  struct CommandAndData cmd2 = {};
  enum UnpackResult res = unpack_command_and_data(buf, len, &cmd2);
  EXPECT_EQ(res, UnpackResult_AllGood);
  EXPECT_EQ(cmd2.command, Command_StartVideo);
  EXPECT_EQ(cmd2.start_video.seconds, video_len);
}




void WriteArmSequenceDataThereAndBack(uint8_t path_id, uint8_t index,
                                      bool take_picture, uint8_t delay,
                                      uint8_t shoulder_yaw,
                                      uint8_t shoulder_pitch,
                                      uint8_t elbow_pitch,
                                      uint8_t wrist_pitch) {
  if (delay > MAX_DELAY_IN_SEQUENCE) {
    delay = MAX_DELAY_IN_SEQUENCE;
  }
  struct CommandAndData cmd = {
      .command = Command_WriteArmSequence,
      .write_arm_sequence =
          {
              .path_id = path_id,
              .index = index,
              .take_picture = take_picture,
              .delay_before_next = delay,
              .target =
                  {
                      .shoulder_yaw = shoulder_yaw,
                      .shoulder_pitch = shoulder_pitch,
                      .elbow_pitch = elbow_pitch,
                      .wrist_pitch = wrist_pitch,
                  },
          },
  };
  uint8_t buf[255] = {0};
  int len = pack_command_and_data(&cmd, buf);
  struct CommandAndData cmd2 = {};
  enum UnpackResult res = unpack_command_and_data(buf, len, &cmd2);

  EXPECT_EQ(res, UnpackResult_AllGood);

  EXPECT_EQ(cmd2.command, Command_WriteArmSequence);

  EXPECT_EQ(cmd2.write_arm_sequence.delay_before_next, delay);
  EXPECT_EQ(cmd2.write_arm_sequence.index, index);
  EXPECT_EQ(cmd2.write_arm_sequence.path_id, path_id);
  EXPECT_EQ(cmd2.write_arm_sequence.take_picture, take_picture);

  EXPECT_EQ(cmd2.write_arm_sequence.target.shoulder_yaw, shoulder_yaw);
  EXPECT_EQ(cmd2.write_arm_sequence.target.shoulder_pitch, shoulder_pitch);
  EXPECT_EQ(cmd2.write_arm_sequence.target.elbow_pitch, elbow_pitch);
  EXPECT_EQ(cmd2.write_arm_sequence.target.wrist_pitch, wrist_pitch);
}

void HasNoUndefinedBehaviorForRandomPacket(std::vector<uint8_t> inp) {
  struct CommandAndData cmd = {.command = Command_StartVideo,
                               .start_video = {.seconds = 0}};
  enum UnpackResult res = unpack_command_and_data(inp.data(), inp.size(), &cmd);
}

FUZZ_TEST(CommandPacking, StartVideoCommandThereAndBack);
FUZZ_TEST(CommandPacking, WriteArmSequenceDataThereAndBack);
FUZZ_TEST(CommandPacking, HasNoUndefinedBehaviorForRandomPacket);