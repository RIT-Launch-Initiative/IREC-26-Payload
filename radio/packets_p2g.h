#include "common.h"
#include "lora.h"
#include <stdbool.h>
#include <stdint.h>

enum P2GPacketType {
  P2GPacketType_LinkTestResponse = 0b00,
  P2GPacketType_LinkControl = 0b01,
  P2GPacketType_CommandResponse = 0b10,
  P2GPacketType_ImageResponse = 0b11,
};

struct P2GLinkHeader {
  enum P2GPacketType packet_type;           // 2 bit
  bool needs_ack;                           // 1 bit
  uint8_t expected_packets_before_response; // 5 bit [0,31] counts down and
                                            //  then the last may have
                                            //  needs_ack set
};

struct ShellReadOutputData {
  uint16_t index;   // maybe MSB means compress it?
  uint8_t buf[128]; // amount filled depends on std(out/err) and which chunk you
                    // are reading
};


enum FlightState {
  FlightState_Booting = 0,
  FlightState_WaitingForLaunch = 1,
  FlightState_ExpectingLaunch = 2,
  FlightState_Flight = 3,
  FlightState_LandedFlipping = 4,
  FlightState_LandedAutomatic = 5,
  FlightState_LandedManualIdle = 6,
  FlightState_LandedManualMoving = 7,
};

struct GeneralStats {
  enum FlightState state;
  uint16_t status_bits;
  uint8_t next_image;
  uint8_t next_command;
};

struct GeneralStatsDetailed {
  struct GeneralStats stats;
  struct ArmTarget arm_position;

  float latitude;
  float longitude;

  int16_t battery_mV;
};


struct Telemetry {
  enum TelemetryType telem_type;
  union {
    struct GeneralStats general_stats;
    struct GeneralStatsDetailed general_stats_detailed;

  };
};