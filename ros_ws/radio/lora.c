#include "lora.h"

int pack_lora_link_change(const struct LoraLinkChange *link_change,
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
  return 6;
}
// buf is at least length 6
enum UnpackResult unpack_lora_link_change(const uint8_t *buf, int len,
                                          struct LoraLinkChange *link_change) {
  if (len < 6) {
    return UnpackResult_TooShort;
  }
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
  return UnpackResult_AllGood;
}
