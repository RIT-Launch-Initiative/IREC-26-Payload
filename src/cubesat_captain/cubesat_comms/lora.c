#include "cubesat_comms/lora.h"

int pack_lora_link_change(const struct LoraLinkChange *link_change,
                          uint8_t *buf) {
  buf[0] = link_change->num_test_packets;
  buf[1] = *(uint8_t*)&link_change->dbm;
  
  buf[2] = (link_change->freq >> 24) & 0xff;
  buf[3] = (link_change->freq >> 16) & 0xff;
  buf[4] = (link_change->freq >> 8) & 0xff;
  buf[5] = (link_change->freq >> 0) & 0xff;
  
  buf[6] = ((link_change->sf & SF_MASK) << SF_POS) |
           ((link_change->bw & BW_MASK) << BW_POS) |
           ((link_change->cr & CR_MASK) << CR_POS);


return 7;
}

// buf is at least length 7
enum UnpackResult unpack_lora_link_change(const uint8_t *buf, int len,
                                          struct LoraLinkChange *link_change) {
  if (len < 7) {
    return UnpackResult_TooShort;
  }
  link_change->num_test_packets= buf[0];
  link_change->dbm = *(int8_t*)&buf[1];

  link_change->freq = 0;
  link_change->freq |= ((uint32_t)buf[2] & 0xff) << 24;
  link_change->freq |= ((uint32_t)buf[3] & 0xff) << 16;
  link_change->freq |= ((uint32_t)buf[4] & 0xff) << 8;
  link_change->freq |= ((uint32_t)buf[5] & 0xff) << 0;


  link_change->sf = (enum SpreadingFactor)((buf[6] >> SF_POS) & SF_MASK);
  link_change->bw = (enum Bandwidth)((buf[6] >> BW_POS) & BW_MASK);
  link_change->cr = (enum CodingRate)((buf[6] >> CR_POS) & CR_MASK);

  return UnpackResult_AllGood;
}
