#include "cubesat_comms/packets_p2g.h"
#include <stdio.h>
#include <string.h>

int pack_p2g_link_header(const struct P2GLinkHeader *header, uint8_t *buf){
  buf[0] = ((header->packet_type & 0b11) << 6) | header->expected_packets_before_response & 0b111111;
  return 1;
}

enum UnpackResult unpack_p2g_link_header(const uint8_t *buf, uint32_t len, struct P2GLinkHeader *header){
  if(len < 1){
    return UnpackResult_TooShort;
  }
  
  enum P2GPacketType typ = 0b11 & (buf[0] >> 6);
  uint8_t left = buf[0] & 0b111111; 
  header->packet_type = typ;
  header->expected_packets_before_response = left;

  return UnpackResult_AllGood;

}

enum UnpackResult unpack_flight_state_and_status(const uint8_t *buf, uint32_t len, struct FlightState *state){
  if (len < SIZEOF_PACKED_FLIGHT_STATE){
    return UnpackResult_TooShort;
  }
  state->phase = buf[0];
  state->status_bits = (buf[1] << 8 | buf[2]);
  return UnpackResult_AllGood;
}

int pack_flight_state_and_status(const struct FlightState *state, uint8_t *buf){
  buf[0] = state->phase;
  buf[1] = (state->status_bits >> 8) & 0xff;
  buf[2] = state->status_bits & 0xff;
  return 3;
}




enum UnpackResult unpack_image_data(const uint8_t *buf, uint32_t len, struct ImageData *header){
  if (len < IMAGE_DATA_SIZE){
    return UnpackResult_TooShort;
  }
  memcpy(header->buf, buf, IMAGE_DATA_SIZE);
  header->image_id = buf[IMAGE_ID_INDEX_IN_PACKET];
  uint8_t msb = buf[IMAGE_BLOCK_ID_INDEX_MSB_IN_PACKET];
  uint8_t lsb = buf[IMAGE_BLOCK_ID_INDEX_LSB_IN_PACKET];
  header->block_index = (msb << 8) | lsb;
  return UnpackResult_AllGood;
}




int pack_landed_heartbeat(const struct LandedHeartbeatStats *stats, uint8_t *buf){
  size_t offset = pack_flight_state_and_status(&stats->state, buf);
  buf[offset] = stats->next_image_id;
  buf[offset+1] = stats->next_exec_id;
  offset+=2;
  offset+=pack_arm_target(&stats->arm_position, buf+offset);
  offset+=pack_int16(stats->battery_mV, buf+offset);
  buf[offset] = stats->motor_temp;
  buf[offset+1]=stats->radio_temp;
  offset+=2;
  return offset;
}

enum UnpackResult unpack_landed_heartbeat(const uint8_t *buf,
                                                uint32_t len,
                                                struct LandedHeartbeatStats *stats){
  if (len < SIZEOF_PACKED_LANDED_HEARTBEAT_STATS){ 
    return UnpackResult_TooShort;
  }
  unpack_flight_state_and_status(buf, len, &stats->state);
  size_t offset = SIZEOF_PACKED_FLIGHT_STATE;
  stats->next_image_id = buf[offset];
  offset++;
  stats->next_exec_id = buf[offset];
  offset++;
  unpack_arm_target(buf+offset, len - offset, &stats->arm_position);
  offset += SIZEOF_PACKED_ARM_TARGET;

  unpack_int16(buf+offset, len-offset, &stats->battery_mV);
  offset+=sizeof(uint16_t);
  stats->motor_temp = buf[offset];
  offset+=sizeof(uint8_t);
  stats->radio_temp = buf[offset];
 
  return UnpackResult_AllGood;
}


int pack_flight_heartbeat(const struct FlightHeartbeatStats *stats, uint8_t *buf){
  size_t offset = pack_flight_state_and_status(&stats->state, buf);
  pack_float(stats->latitude, buf+offset);
  offset+=sizeof(float);
  pack_float(stats->longitude, buf+offset);
  offset+=sizeof(float);
  pack_uint16(stats->altitude, buf+offset);
  offset+=sizeof(uint16_t);
  pack_uint16(stats->s_since_boost, buf+offset);
  offset+=sizeof(uint16_t);
  pack_uint16(stats->battery_mV, buf+offset);
  offset+=sizeof(int16_t);
  buf[offset] = stats->radio_temp;  
  return offset+1;
}


enum UnpackResult unpack_flight_heartbeat(const uint8_t *buf,
                                                uint32_t len,
                                                struct FlightHeartbeatStats *stats){
  if (len < SIZEOF_PACKED_FLIGHT_HEARTBEAT_STATS){
    return UnpackResult_TooShort;
  }
  
  unpack_flight_state_and_status(buf, len, &stats->state);
  printf("states: %d bits %04x\n", stats->state.phase, stats->state.status_bits);
  size_t offset = SIZEOF_PACKED_FLIGHT_STATE;

  unpack_float(buf+offset, len-offset, &stats->latitude);
  offset+=sizeof(float);

  unpack_float(buf+offset, len-offset, &stats->longitude);
  offset+=sizeof(float);

  unpack_uint16(buf+offset, len-offset, &stats->altitude);
  offset+=sizeof(uint16_t);

  unpack_uint16(buf+offset, len-offset, &stats->s_since_boost);
  offset+=sizeof(uint16_t);

  unpack_int16(buf+offset, len-offset, &stats->battery_mV);
  offset+=sizeof(int16_t);

  stats->radio_temp = buf[offset];

  return UnpackResult_AllGood;
}


int pack_telemetry(const struct Telemetry *telem, uint8_t *buf){
  buf[0] = telem->telem_type;
  switch (telem->telem_type){

  case TelemetryType_FlightHeartbeat:
    return 1+pack_flight_heartbeat(&telem->flight_heartbeat_stats, buf+1);
  case TelemetryType_LandedHeartbeat:
    return 1+pack_landed_heartbeat(&telem->landed_heartbeat_stats, buf+1);
  case TelemetryType_Actuators:
    return 1+pack_actuator_positions(&telem->actuators, buf+1);
  case TelemetryType_GNSS:
  case TelemetryType_System:
  case TelemetryType_Orientations:
  case TelemetryType_Temps:
  case TelemetryType_Power:
    break;
  }

  return 1;
}

int pack_command_response(const struct CommandResponse *cmd, uint8_t *buf){
  buf[0] = cmd->cmd;
  switch (cmd->cmd){
  case Command_TelemetryRequest:
    return 1+pack_telemetry(&cmd->telemetry, buf+1);
  default:
    return 1;
  }
  return 1;
}

enum UnpackResult unpack_command_response(const uint8_t *buf, uint32_t len, struct CommandResponse *resp){
  if (len < 1){
    return UnpackResult_TooShort;
  }

  resp->cmd = buf[0];
  switch (resp->cmd){
    // 0 length messages
    case Command_ForceFlight:
    case Command_ExpectFlight:
    case Command_UnexpectFlight:
    case Command_ForceManual:
    case Command_ZeroShoulder_AssumeOpen:
    case Command_RunOpenSequence:
      return UnpackResult_AllGood;
    // messages of length
    case Command_TelemetryRequest:
      return unpack_telemetry(buf+1, len-1, &resp->telemetry);
    case Command_StartVideo:
    case Command_StopVideo:
    case Command_ReCrop:
    case Command_SendArmTargetAndComeBack:
    case Command_SendArmTargetForPhotoAndComeBack:
    case Command_SendIdlePosition:
    case Command_ShellExec:
    case Command_ShellExecInfo:
    case Command_ShellReadStdout:
    case Command_ShellReadStderr:
    case Command_ClearFlightDANGER:
    case Command_GetFlightNumber:
    case Command_Callsign:

    case Command_WriteArmSequence:
    case Command_ReadArmSequence:
    case Command_ExecuteArmSequence:
    case Command_CancelExecutingArmSequence:
      return UnpackResult_Unimplemented;
      break;
    case Command_MaxCommand:
      return UnpackResult_UnknownCommand;
    }

  return UnpackResult_UnknownCommand;

}



int pack_actuator_positions(const struct ActuatorPositions *pos, uint8_t *buf){
  int len = pack_arm_target(&pos->arms, buf);
  buf[4] = pos->servo1;
  buf[5] = pos->servo2;
  buf[6] = pos->servo3;
  buf[7] = pos->servo4;
  return len+4;
}
enum UnpackResult unpack_actuator_positions(const uint8_t *buf,
                                            uint32_t len,
                                            struct ActuatorPositions *target){
  if (len < 8){
    return UnpackResult_TooShort;
  }
  unpack_arm_target(buf, len, &target->arms);
  target->servo1 = buf[4];
  target->servo2 = buf[5];
  target->servo3 = buf[6];
  target->servo4 = buf[7];
  return UnpackResult_AllGood;
}





enum UnpackResult unpack_telemetry(const uint8_t *buf, uint32_t len, struct Telemetry *telem){
  if (len < 1){
    return UnpackResult_TooShort;
  }
  telem->telem_type = buf[0];
switch (telem->telem_type){
  case TelemetryType_FlightHeartbeat:
    return unpack_flight_heartbeat(buf+1, len-1, &telem->flight_heartbeat_stats);
  case TelemetryType_LandedHeartbeat:
    return unpack_landed_heartbeat(buf+1, len-1, &telem->landed_heartbeat_stats);
  case TelemetryType_Actuators:
    return unpack_actuator_positions(buf+1, len-1, &telem->actuators);
  case TelemetryType_GNSS:
    
  case TelemetryType_System:
  case TelemetryType_Orientations:
  case TelemetryType_Temps:
  case TelemetryType_Power:
    return UnpackResult_UnknownCommand;
  }
}