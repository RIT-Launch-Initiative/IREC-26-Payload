# feature/binary-flight-logger

## What

A Tier-1 binary flight data logger that records the core flight picture
outside of rosbag, as a new `cubesat_flight_logger` package:

### Library (`flight_logger.hpp` / `flight_logger.cpp`) — no ROS dependencies
- Fixed 51-byte packed record: timestamp (uint64 ns), accel xyz (float32),
  GPS lat/lon/alt (float64), battery voltage (float32), flight state (uint8),
  arm status flags (uint16; low byte = boolean flags, bits 8-11 = arm state).
  A 12-byte header (magic `ATLASLOG`, version, record size) starts each file.
- Records accumulate in a pre-allocated ring buffer (default 64 records) that
  is written through when full; `fsync` every N records (configurable,
  default 50 — about one second of data at the 50 Hz accel rate).
- File opened `O_WRONLY | O_CREAT | O_APPEND`, never seeks; the header is only
  written when the file is empty, so a node respawn appends to the same log
  instead of corrupting it. Writes handle EINTR/short writes; failures are
  counted, never thrown.

### Node (`flight_logger_node`)
- Subscribes to `pi/lis3dh`, `pi/gps`, `pi/power`, `pi/flight_state`,
  `stm/arm_status`. One record is written per accel sample (the 50 Hz
  driver); other topics refresh the latest values folded into each record.
- Logs to `<flight_dir>/flight_log.bin` (`log_file_name`,
  `fsync_every_records` parameters). A 5 s timer flushes the buffer tail so a
  quiet system still hits the disk.
- On SIGTERM/SIGINT, rclcpp's default signal handler ends spin and the node
  destructor (and the library destructor as a backstop) flushes and fsyncs.
- Added to `flight.launch.py` with respawn like the other nodes.

### Decoder (`tools/decode_flight_log.py`)
- Stdlib-only; validates magic/version/record size, emits CSV with symbolic
  flight-state and arm-flag names, and tolerates a truncated final record
  (power cut) with a warning instead of dying.

## Why

rosbag is the rich record, but it is also the most fragile thing on the Pi
(mcap + zstd + DDS all in the write path, and it has its own splitting/
compression machinery). When the payload hits the desert at speed, a dumb
append-only file with bounded fsync lag is the thing most likely to survive
and be readable. 51 bytes x 50 Hz is ~2.5 KB/s — irrelevant on the SD card.

## Testing

- Built clean in dev:humble.
- Round-trip test: 123 records written through the library (fsync_every=10,
  8-record buffer to force buffer-wrap paths), file closed, reopened, 1 more
  record appended. Decoder output verified: 124 records, correct values,
  single header, file size exactly 12 + 124*51 bytes.
