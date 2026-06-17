#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#define MAX_PACKETS_BEFORE_RESPONSE 63
#define SHELL_OUTPUT_CHUNK_SIZE 128
#define IMAGE_DATA_SIZE 128
#define IMAGE_ID_INDEX_IN_PACKET 6
#define IMAGE_BLOCK_ID_INDEX_MSB_IN_PACKET 7
#define IMAGE_BLOCK_ID_INDEX_LSB_IN_PACKET 8

#define MAX(a, b) (a > b ? a : b)
#define MAX3(a, b, c) MAX(MAX(a, b), c)
#define MAX4(a, b, c, d) MAX(MAX3(a, b, c), d)
#define MAX5(a, b, c, d, e) MAX(MAX4(a, b, c, d), e)
#define MAX6(a, b, c, d, e, f) MAX(MAX5(a, b, c, d, e), f)
#define MAX7(a, b, c, d, e, f, g) MAX(MAX6(a, b, c, d, e, f), g)

enum UnpackResult {
    UnpackResult_TooShort,
    UnpackResult_TooLong,
    UnpackResult_UnknownCommand,
    UnpackResult_NoCommand,

    UnpackResult_AllGood,
    UnpackResult_Unimplemented,

};

int pack_int16(int16_t f, uint8_t *buf);
enum UnpackResult unpack_int16(const uint8_t *buf, uint32_t len, int16_t *i);

int pack_uint16(uint16_t u, uint8_t *buf);
enum UnpackResult unpack_uint16(const uint8_t *buf, uint32_t len, uint16_t *u);

int pack_uint32(uint32_t u, uint8_t *buf);
enum UnpackResult unpack_uint32(const uint8_t *buf, uint32_t len, uint32_t *u);

int pack_float(float f, uint8_t *buf);
enum UnpackResult unpack_float(const uint8_t *buf, uint32_t len, float *f);

struct v3int8 {
    int8_t x;
    int8_t y;
    int8_t z;
};
struct v3int16 {
    int16_t x;
    int16_t y;
    int16_t z;
};
#define SIZEOF_PACKED_V3INT16 6
int pack_v3int16(const struct v3int16 *f, uint8_t *buf);
enum UnpackResult unpack_v3int16(const uint8_t *buf, uint32_t len, struct v3int16 *f);

struct BatteryTelemetry {
    int16_t voltage;  // mV
    uint16_t current; // mA
};

struct ArmAbsoluteTelemetryLowRes {
    struct v3int8 base_accel;
    struct v3int8 upper_arm_accel;
    struct v3int8 forearm_accel;
};

struct ArmTarget {
    int8_t shoulder_yaw;
    int8_t shoulder_pitch;
    int8_t elbow_pitch;
    int8_t wrist_pitch;
};
#define SIZEOF_PACKED_ARM_TARGET 4

int pack_arm_target(const struct ArmTarget *target, uint8_t *buf);
enum UnpackResult unpack_arm_target(const uint8_t *buf, uint32_t len, struct ArmTarget *target);

struct PhotoTransform {
    // 1. select a box of left,right,top,bottom
    uint16_t left;
    uint16_t right;
    uint16_t top;
    uint16_t bottom;

    // 2. resize that box such that the width is this much
    uint16_t output_width;

    // 3. encode to this quality
    uint8_t quality; // 0-7

    // quality and output_width packed into one
    // max width 8192 (would take years to transmit)
    // qqqwwwwwwwwwwwww
};
#define SIZEOF_PACKED_PHOTOTRANSFORM (2 * 5)
enum UnpackResult unpack_phototransform(const uint8_t *buf, uint32_t len, struct PhotoTransform *tform);
int pack_phototransform(const struct PhotoTransform *tform, uint8_t *buf);

// vcgencmd measure_temp
// vcgencmd measure_volts
enum TelemetryType {
    TelemetryType_FlightHeartbeat, // GPS Data, acceleration
    TelemetryType_LandedHeartbeat, // FlightState, StatusBitset, Next Image ID, Next
                                   // Command ID
    TelemetryType_Actuators,       // Angles of joints, Angles of Servos, Stalls, Low
                                   // precision Accelerometer readings
    TelemetryType_GNSS,            // Lat Long Altitude FixType Satellites
    TelemetryType_System,          // CPU/RAM/Storage/Bootcount%256  (top -b -n 1 | head -n
                                   // 5 , free, df)
    TelemetryType_Orientations,    // High quality Accel. readings from all over
    TelemetryType_Temps,           // Temperatures from all over (CPU, STM, Board1, Board2,
                                   // Ambient)
    TelemetryType_Power,           // Battery Voltage, BatteryCurrent, Enabled Regulators
};

// From GS to Payload
enum Command {
    Command_ForceManual, // stop anything on a timer and listen for radio message
    Command_ForceFlight, // in case boost detect failed
    Command_ExpectFlight, // start going faster and sending me more packets for status of flight bc you're about to take
                          // off. maybe also turn on cameras

    Command_BackToPad, // go back to normal waitin on the pad mode
    Command_StartAutoFlipping,
    Command_UnfoldArm,
    Command_Panorama,
    Command_StartVideo, // Start Runcam  video TODO
    Command_StopVideo,  // Stop Runcam video TODO
    Command_TakePicture,
    Command_ImageMetadata,
    Command_ReCrop, // refer to original image for an image id, recrop it, save that as the next image_id (rather than
    // the next image id going to a newly taken photo)
    Command_SendArmTarget,
    Command_SendArmTargetAndComeBack, // go to this position and go back to rest
    // position after. Don't take a picture
    Command_SendArmTargetForPhotoAndComeBack, // go to this position, take a
    // picture, return to transmit
    // position
    Command_SendIdlePosition, // treat the sent position as the 'home'
    // position for sending UNLESS you don't
    // hear back
    Command_SetShoulder, // tell the stm the position of all the joints

    Command_JogMotor,
    Command_MoveServo,

    Command_NewFlightDanger, // DANGER delete the 'we launched' flag and reset
    Command_Restart,
    // counters and delete directories for images and
    // shell commands. Restarts all programs, may blink
    // offline for a sec
    Command_TelemetryRequest, // Get really detailed info about a subsystem, can
    // be 'responded' to w/o being requested

    Command_Callsign,
    Command_MaxCommand,
};

const char *telemetry_type_to_str(enum TelemetryType typ);

#ifdef __cplusplus
}
#endif
