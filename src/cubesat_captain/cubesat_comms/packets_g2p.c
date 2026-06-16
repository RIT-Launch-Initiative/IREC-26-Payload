#include "cubesat_comms/packets_g2p.h"
#include "string.h"

enum UnpackResult unpack_g2p_link_header(const uint8_t *buf, uint32_t len, struct G2PLinkHeader *header) {
    if (len < 1) {
        return UnpackResult_TooShort;
    }

    enum G2PPacketType typ = 0b11 & (buf[0] >> 6);
    uint8_t left = buf[0] & 0b111111;
    header->packet_type = typ;
    header->expected_packets_before_response = left;

    return UnpackResult_AllGood;
}
int pack_g2p_link_header(const struct G2PLinkHeader *header, uint8_t *buf) {
    buf[0] = ((header->packet_type & 0b11) << 6) | (header->expected_packets_before_response & 0b111111);
    return 1;
}
int pack_arm_target(const struct ArmTarget *target, uint8_t *buf) {
    buf[0] = *(uint8_t*)&target->shoulder_yaw;
    buf[1] = *(uint8_t*)&target->shoulder_pitch;
    buf[2] = *(uint8_t*)&target->elbow_pitch;
    buf[3] = *(uint8_t*)&target->wrist_pitch;
    return 4;
}
enum UnpackResult unpack_arm_target(const uint8_t *buf, uint32_t len, struct ArmTarget *target) {
    if (len < 4) {
        return UnpackResult_TooShort;
    }

    target->shoulder_yaw =  *(int8_t*)&buf[0];
    target->shoulder_pitch =*(int8_t*)&buf[1];
    target->elbow_pitch =   *(int8_t*)&buf[2];
    target->wrist_pitch =   *(int8_t*)&buf[3];

    return UnpackResult_AllGood;
}


int pack_motor_jog(const struct MotorJog *jog, uint8_t *buf){
  buf[0] = jog->motor_id;
  buf[1] = jog->duration_ticks;
  pack_int16(jog->millivolts, buf+2);
  
  return SIZEOF_PACKERD_MOTOR_JOG;
}

enum UnpackResult unpack_motor_jog(const uint8_t *buf, int len, struct MotorJog *jog){
  if (len < SIZEOF_PACKERD_MOTOR_JOG){
    return UnpackResult_TooShort;
  }
  jog->motor_id = buf[0];
  jog->duration_ticks = buf[1];
  unpack_int16(buf+2, len-2, &jog->millivolts);

  return UnpackResult_AllGood;
}

int pack_servo_motion(const struct ServoMotion *motion, uint8_t *buf){
  buf[0] = motion->openness;
  buf[1] = motion->open_travel_time;
  buf[2] = motion->open_time;
  buf[3] = motion->close_travel_time;
  buf[4] = motion->closeness;
  buf[5] = motion->which_servo;
  return SIZEOF_PACKED_SERVO_MOTION;
}
enum UnpackResult unpack_servo_motion(const uint8_t *buf, int len, struct ServoMotion *motion){
  if (len < SIZEOF_PACKERD_MOTOR_JOG){
    return UnpackResult_TooShort;
  }
  motion->openness = buf[0];
  motion->open_travel_time = buf[1];
  motion->open_time = buf[2];
  motion->close_travel_time = buf[3];
  motion->closeness = buf[4];
  motion->which_servo= buf[5];

  return UnpackResult_AllGood;

}


static const uint32_t take_picture_location = 7;
static const uint8_t take_picture_mask = 1 << take_picture_location;


int pack_command_and_data(const struct CommandAndData *cmd_and_data, uint8_t *buf) {
    buf[0] = (uint8_t)cmd_and_data->command;
    switch (cmd_and_data->command) {
    case Command_ForceManual:
    case Command_ForceFlight:
    case Command_ExpectFlight:
    case Command_BackToPad:
    case Command_StartAutoFlipping:
    case Command_UnfoldArm:
    case Command_Panorama:
    case Command_NewFlightDanger:
    case Command_StopVideo:
        return 1;
    case Command_Callsign:
        pack_callsign(&cmd_and_data->callsign, buf + 1);
        return 1 + SIZEOF_PACKED_CALLSIGN;
    case Command_StartVideo:
        buf[1] = cmd_and_data->start_video.seconds;
        return 2;
    case Command_TakePicture:
        return 1 + pack_phototransform(&cmd_and_data->take_picture, buf + 1);

    case Command_ImageMetadata:
        buf[1] = cmd_and_data->metadata_ask_image_id;
        return 2;
    case Command_ReCrop:
        buf[1] = cmd_and_data->recrop.original_image;
        pack_phototransform(&cmd_and_data->recrop.transform, buf + 1);
        return 1 + SIZEOF_PACKED_RECROP_DATA;

    case Command_SendArmTarget:
        return 1 + pack_arm_target(&cmd_and_data->send_arm_to_target, &buf[1]);
    case Command_SendArmTargetAndComeBack:
        return 1 + pack_arm_target(&cmd_and_data->send_arm_to_target_and_come_back, &buf[1]);
    case Command_SendArmTargetForPhotoAndComeBack:
        return 1 + pack_arm_target(&cmd_and_data->send_arm_to_target_for_photo_and_come_back, &buf[1]);

    case Command_SetShoulder:
        return 1 + pack_arm_target(&cmd_and_data->set_shoulder_position, &buf[1]);

    case Command_JogMotor:
      return 1+pack_motor_jog(&cmd_and_data->motor_jog, &buf[1]);

    case Command_MoveServo:
      return 1+pack_servo_motion(&cmd_and_data->servo_motion, &buf[1]);

    case Command_SendIdlePosition:
        return 1 + pack_arm_target(&cmd_and_data->send_idle_position, &buf[1]);
    case Command_TelemetryRequest:
        buf[1] = cmd_and_data->telem_request.telem_type;
        return 2;
    case Command_MaxCommand:
        // WARNING - invalid command: unpack will tell them but this is weird
        return 1;
    }
    return 1;
}

enum UnpackResult unpack_command_and_data(const uint8_t *buf, int len, struct CommandAndData *cmd_and_data) {
    if (len < 1) {
        return UnpackResult_NoCommand;
    }
    cmd_and_data->command = (enum Command)buf[0];
    if (cmd_and_data->command >= Command_MaxCommand) {
        return UnpackResult_UnknownCommand;
    }
    int data_len = len - 1;
    const uint8_t *data_buf = &buf[1];
    switch (cmd_and_data->command) {
    case Command_ForceManual:
    case Command_ForceFlight:
    case Command_ExpectFlight:
    case Command_BackToPad:
    case Command_StartAutoFlipping:
    case Command_UnfoldArm:
    case Command_Panorama:
    case Command_NewFlightDanger:
    case Command_StopVideo:
        return UnpackResult_AllGood; // no data with these messages
    case Command_StartVideo:
        if (data_len >= 1) {
            cmd_and_data->start_video.seconds = data_buf[0];
            return UnpackResult_AllGood;
        } else {
            return UnpackResult_TooShort;
        }
    case Command_TakePicture:
        return unpack_phototransform(data_buf, data_len, &cmd_and_data->take_picture);
    case Command_ImageMetadata:
      if (data_len>=1){
        cmd_and_data->metadata_ask_image_id = data_buf[0];
        return UnpackResult_AllGood;
      } else {
        return UnpackResult_TooShort;
      }
    case Command_ReCrop:
        return UnpackResult_Unimplemented;

    case Command_SendArmTarget:
        return unpack_arm_target(data_buf, data_len, &cmd_and_data->send_arm_to_target_and_come_back);
    case Command_SendArmTargetAndComeBack:
        return unpack_arm_target(data_buf, data_len, &cmd_and_data->send_arm_to_target);
    case Command_SendArmTargetForPhotoAndComeBack:
        return unpack_arm_target(data_buf, data_len, &cmd_and_data->send_arm_to_target_for_photo_and_come_back);
    case Command_SendIdlePosition:
        return unpack_arm_target(data_buf, data_len, &cmd_and_data->send_idle_position);

    case Command_SetShoulder:
        return unpack_arm_target(data_buf, data_len, &cmd_and_data->set_shoulder_position);


    case Command_MoveServo:
      return unpack_servo_motion(data_buf, data_len, &cmd_and_data->servo_motion);
    case Command_JogMotor:
      return unpack_motor_jog(data_buf, data_len, &cmd_and_data->motor_jog);
    case Command_TelemetryRequest:
        if (data_len > 0) {
            cmd_and_data->telem_request.telem_type = data_buf[0];
            return UnpackResult_AllGood;
        } else {
            return UnpackResult_TooShort;
        }
    case Command_Callsign:{
      memcpy(cmd_and_data->callsign.buf, data_buf, sizeof(cmd_and_data->callsign.buf));
      return UnpackResult_AllGood;
    } break;
    case Command_MaxCommand:
        return UnpackResult_UnknownCommand;
    }
    return UnpackResult_UnknownCommand;
}

int pack_int16(int16_t i, uint8_t *buf) {
    memcpy(buf, &i, sizeof(int16_t));
    return sizeof(int16_t);
}
enum UnpackResult unpack_int16(const uint8_t *buf, uint32_t len, int16_t *i) {
    if (len < 2) {
        return UnpackResult_TooShort;
    }

    memcpy(i, buf, sizeof(int16_t));

    return UnpackResult_AllGood;
}

int pack_uint16(uint16_t i, uint8_t *buf) {
    memcpy(buf, &i, sizeof(uint16_t));
    return sizeof(uint16_t);
}
enum UnpackResult unpack_uint16(const uint8_t *buf, uint32_t len, uint16_t *i) {
    if (len < 2) {
        return UnpackResult_TooShort;
    }

    memcpy(i, buf, sizeof(uint16_t));

    return UnpackResult_AllGood;
}

int pack_uint32(uint32_t i, uint8_t *buf) {
    memcpy(buf, &i, sizeof(uint32_t));
    return sizeof(uint32_t);
}
enum UnpackResult unpack_uint32(const uint8_t *buf, uint32_t len, uint32_t *i) {
    if (len < 4) {
        return UnpackResult_TooShort;
    }

    memcpy(i, buf, sizeof(uint32_t));

    return UnpackResult_AllGood;
}




int pack_float(float f, uint8_t *buf) {
    memcpy(buf, &f, sizeof(float));
    return sizeof(float);
}
enum UnpackResult unpack_float(const uint8_t *buf, uint32_t len, float *f) {
    if (len < 4) {
        return UnpackResult_TooShort;
    }

    memcpy(f, buf, sizeof(float));

    return UnpackResult_AllGood;
}

const char *telemetry_type_to_str(enum TelemetryType typ) {
    switch (typ) {
    case TelemetryType_LandedHeartbeat:
        return "General";
    case TelemetryType_Actuators:
        return "Actuators";
    case TelemetryType_GNSS:
        return "GNSS";
    case TelemetryType_System:
        return "System";
    case TelemetryType_Orientations:
        return "Orientations";
    case TelemetryType_Temps:
        return "Temp";
    case TelemetryType_Power:
        return "Power";
    default:
        return "Unknown";
    }
}

void pack_callsign(const struct Callsign *callsign, uint8_t *buf) { memcpy(buf, callsign->buf, 6); }


int pack_phototransform(const struct PhotoTransform *tform, uint8_t *buf){
  uint32_t size = pack_uint16(tform->left, buf);
  size+=pack_uint16(tform->right, buf + size);
  size+=pack_uint16(tform->top, buf + size);
  size+=pack_uint16(tform->bottom, buf + size);
  
  uint16_t packed_q_and_width = ((tform->quality & 0b111) << 13) | (tform->output_width & 0b1111111111111);
  size+=pack_uint16(packed_q_and_width, buf + size);
  return size;
}


enum UnpackResult unpack_phototransform(const uint8_t *buf,
                                        uint32_t len,
                                        struct PhotoTransform *tform){
  if (len< SIZEOF_PACKED_PHOTOTRANSFORM){
    return UnpackResult_TooShort;
  }
  unpack_uint16(buf, 2, &tform->left);
  unpack_uint16(buf+2, 2, &tform->right);
  unpack_uint16(buf+4, 2, &tform->top);
  unpack_uint16(buf+6, 2, &tform->bottom);
  uint16_t packed = 0;  
  unpack_uint16(buf+8, 2, &packed);
  tform->quality = (packed >> 13) & 0b111;
  tform ->output_width = packed & 0b1111111111111;
  return UnpackResult_AllGood;

}



int pack_image_block_request(const struct ImageBlockRequest *req, uint8_t *buf){
  buf[0] = req->image_id;
  buf[1] = req->num;
  if (req->num > MAX_BLOCKS_PER_REQUEST){
    buf[1] = MAX_BLOCKS_PER_REQUEST;
  }
  size_t off =2;
  for (uint8_t i = 0; i < buf[1]; i++){
    off+=pack_uint16(req->block_ids[i], buf+off);
  }
  return off;
}
enum UnpackResult unpack_image_block_request(uint8_t *buf, uint32_t len, struct ImageBlockRequest *req){
  if (len < 2){
    return UnpackResult_TooShort;
  }
  req->image_id = buf[0];
  req->num = buf[1];
  if (len < (uint32_t)(req->num * 2 + 2)){
    return UnpackResult_TooShort;
  }
  if (req->num > MAX_BLOCKS_PER_REQUEST){
    return UnpackResult_TooLong;
  }
  uint32_t offset = 2;
  for (uint8_t i = 0; i < req->num; i++){
    unpack_uint16(buf+offset, len-offset, &req->block_ids[i]);
    offset+=2;
  }

  return UnpackResult_AllGood;
}
