#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "common.h"

enum SpreadingFactor {
  SF_5 = 0b000,
  SF_6 = 0b001,
  SF_7 = 0b010,
  SF_8 = 0b011,
  SF_9 = 0b100,
  SF_10 = 0b101,
  SF_11 = 0b110,
  SF_12 = 0b111,
};

enum Bandwidth {
  BW_62_5 = 0b00,
  BW_125 = 0b01,
  BW_250 = 0b10,
  BW_500 = 0b11,
};

enum CodingRate {
  CR_4_5 = 0b00,
  CR_4_6 = 0b01,
  CR_4_7 = 0b10,
  CR_4_8 = 0b11,
};

#define SF_POS 0
#define SF_MASK 0b111
#define BW_POS 3
#define BW_MASK 0b11
#define CR_POS 5
#define CR_MASK 0b11

struct LoraLinkChange {
  bool is_request;  // 1 is request, 0 is response
  bool is_accepted; // 1 is yes i will switch over, 0 is no im staying here,
                    // only valid if is_response
  uint8_t num_test_packets; // max [0,63]

  enum SpreadingFactor sf;
  enum Bandwidth bw;
  enum CodingRate cr;
  uint32_t freq;
};

// buf is at least length 6
int pack_lora_link_change(const struct LoraLinkChange *link_change,
                           uint8_t *buf);
// buf is at least length 6
enum UnpackResult unpack_lora_link_change(const uint8_t *buf, int len, struct LoraLinkChange *link_change);