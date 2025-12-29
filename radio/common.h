#include "lora.h"
#include <stdbool.h>
#include <stdint.h>
#define MAX_PACKETS_BEFORE_RESPONSE 31

struct v3int8 {
  int8_t x;
  int8_t y;
  int8_t z;
};

struct BatteryTelemetry {
  int16_t voltage; // mV
  uint16_t current; // mA
};

struct ArmAbsoluteTelemetryLowRes {
  struct v3int8 base_accel;
  struct v3int8 upper_arm_accel;
  struct v3int8 forearm_accel;
};

struct ArmTarget {
  uint8_t shoulder_yaw;
  uint8_t shoulder_pitch;
  uint8_t elbow_pitch;
  uint8_t wrist_pitch;
};

// vcgencmd measure_temp
// vcgencmd measure_volts
enum TelemetryType {
  TelemetryType_General,     // FlightState, StatusBitset, Next Image ID, Next
                             // Command ID
  TelemetryType_GeneralLong, // FlightState, StatusBitset, Next Image ID, Next
                             // Command ID, JointPositions, Lat, Long, Altitude,
                             // Important Temps, Battery Voltage
  TelemetryType_Actuators,   // Angles of joints, Angles of Servos, Stalls, Low
                             // precision Accelerometer readings
  TelemetryType_GNSS,        // Lat Long Altitude Fix
  TelemetryType_System, // CPU/RAM/Storage/Bootcount%256  (top -b -n 1 | head -n
                        // 5 , free, df)
  TelemetryType_Orientations, // High quality Accel. readings from all over
  TelemetryType_Temps, // Temperatures from all over (CPU, STM, Board1, Board2,
                       // Ambient)
  TelemetryType_Power, // Battery Voltage, BatteryCurrent, Enabled Regulators
};

