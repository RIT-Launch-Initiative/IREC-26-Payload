#include <stdbool.h>
#include <stdint.h>

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
void pack_lora_link_change(const struct LoraLinkChange *link_change,
                           uint8_t *buf) {
  buf[0] = (link_change->is_request << 7) | (link_change->is_accepted << 6) |
           (link_change->num_test_packets & 0b111111);
  buf[1] = ((link_change->sf & SF_MASK) << SF_POS) |
           ((link_change->bw & BW_MASK) << BW_POS) |
           ((link_change->cr & CR_MASK) << CR_POS);

  buf[2] = (link_change->freq >> 24) & 0xff;
  buf[3] = (link_change->freq >> 16) & 0xff;
  buf[4] = (link_change->freq >> 8) & 0xff;
  buf[5] = (link_change->freq >> 0) & 0xff;
}
// buf is at least length 6
void unpack_lora_link_change(uint8_t *buf, struct LoraLinkChange *link_change) {
  link_change->is_request = (buf[0] >> 7) & 0b1;
  link_change->is_request = (buf[0] >> 6) & 0b1;
  link_change->is_request = (buf[0] & 0b111111);
  link_change->sf = (enum SpreadingFactor)((buf[1] >> SF_POS) & SF_MASK);
  link_change->bw = (enum Bandwidth)((buf[1] >> BW_POS) & BW_MASK);
  link_change->cr = (enum CodingRate)((buf[1] >> CR_POS) & CR_MASK);

  link_change->freq = 0;
  link_change->freq |= ((uint32_t)buf[2] << 24) & 0xff;
  link_change->freq |= ((uint32_t)buf[3] << 16) & 0xff;
  link_change->freq |= ((uint32_t)buf[4] << 8) & 0xff;
  link_change->freq |= ((uint32_t)buf[5] << 0) & 0xff;
}
