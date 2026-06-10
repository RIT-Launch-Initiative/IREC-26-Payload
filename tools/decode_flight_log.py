#!/usr/bin/env python3
"""Decode an ATLAS Tier-1 binary flight log to CSV.

Usage:
    ./decode_flight_log.py flight_log.bin            # CSV to stdout
    ./decode_flight_log.py flight_log.bin -o out.csv

File layout (see cubesat_flight_logger/flight_logger.hpp):
    header:  8s magic "ATLASLOG", uint16 version, uint16 record_size
    records: uint64 timestamp_ns,
             3x float32 accel xyz (m/s^2),
             3x float64 lat/lon/alt,
             float32 battery (V),
             uint8 flight state,
             uint16 arm status flags
All little-endian, packed.
"""
import argparse
import csv
import struct
import sys

HEADER_FMT = "<8sHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECORD_FMT = "<Q3f3dfBH"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
MAGIC = b"ATLASLOG"
SUPPORTED_VERSION = 1

FLIGHT_STATES = {
    0: "PAD",
    1: "PREBOOST",
    2: "FLIGHT",
    3: "FLIPPING",
    4: "UNFOLDING",
    5: "AUTO_CAMERA",
    6: "MANUAL_CONTROL",
    7: "EMERGENCY",
}

ARM_FLAG_NAMES = [
    (1 << 0, "booted"),
    (1 << 1, "move_failed"),
    (1 << 2, "wrist_servo_en"),
    (1 << 3, "flip_servo_en"),
    (1 << 4, "motor_en"),
    (1 << 5, "cant_trust_imu_link"),
    (1 << 6, "encoders_not_updating"),
]

COLUMNS = [
    "timestamp_ns",
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",
    "latitude",
    "longitude",
    "altitude_m",
    "battery_v",
    "flight_state",
    "flight_state_name",
    "arm_status_flags",
    "arm_state",
    "arm_flags",
]


def decode(path, out):
    writer = csv.writer(out)
    writer.writerow(COLUMNS)

    with open(path, "rb") as f:
        header = f.read(HEADER_SIZE)
        if len(header) < HEADER_SIZE:
            sys.exit(f"{path}: file too short for header")
        magic, version, record_size = struct.unpack(HEADER_FMT, header)
        if magic != MAGIC:
            sys.exit(f"{path}: bad magic {magic!r} (not an ATLAS flight log)")
        if version != SUPPORTED_VERSION:
            sys.exit(f"{path}: unsupported version {version}")
        if record_size != RECORD_SIZE:
            sys.exit(f"{path}: record size {record_size} != expected {RECORD_SIZE}")

        count = 0
        while True:
            blob = f.read(RECORD_SIZE)
            if len(blob) == 0:
                break
            if len(blob) < RECORD_SIZE:
                print(
                    f"warning: {len(blob)} trailing bytes (truncated final record, likely power cut)",
                    file=sys.stderr,
                )
                break
            (ts, ax, ay, az, lat, lon, alt, batt, state, arm_flags) = struct.unpack(RECORD_FMT, blob)
            arm_state = (arm_flags >> 8) & 0x0F
            flag_names = "|".join(name for bit, name in ARM_FLAG_NAMES if arm_flags & bit)
            writer.writerow(
                [
                    ts,
                    f"{ax:.5f}",
                    f"{ay:.5f}",
                    f"{az:.5f}",
                    f"{lat:.7f}",
                    f"{lon:.7f}",
                    f"{alt:.2f}",
                    f"{batt:.3f}",
                    state,
                    FLIGHT_STATES.get(state, f"UNKNOWN_{state}"),
                    arm_flags,
                    arm_state,
                    flag_names,
                ]
            )
            count += 1

    print(f"decoded {count} records", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Decode an ATLAS binary flight log to CSV")
    parser.add_argument("logfile", help="path to flight_log.bin")
    parser.add_argument("-o", "--output", help="output CSV path (default: stdout)")
    args = parser.parse_args()

    if args.output:
        with open(args.output, "w", newline="") as out:
            decode(args.logfile, out)
    else:
        decode(args.logfile, sys.stdout)


if __name__ == "__main__":
    main()
