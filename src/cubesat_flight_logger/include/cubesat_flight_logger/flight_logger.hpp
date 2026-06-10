#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cubesat_flight_logger {

// On-disk record. Keep in sync with tools/decode_flight_log.py ("<Q3f3dfBH").
struct __attribute__((packed)) FlightLogRecord {
    uint64_t timestamp_ns;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    double latitude;
    double longitude;
    double altitude_m;
    float battery_v;
    uint8_t flight_state;
    uint16_t arm_status_flags;
};
static_assert(sizeof(FlightLogRecord) == 51, "record layout must match the decoder");

// File header written once at the start of a new file.
struct __attribute__((packed)) FlightLogHeader {
    char magic[8]; // "ATLASLOG"
    uint16_t version;
    uint16_t record_size;
};
static_assert(sizeof(FlightLogHeader) == 12, "header layout must match the decoder");

constexpr uint16_t FLIGHT_LOG_VERSION = 1;

/**
 * Tier-1 flight data logger. Standalone (no ROS dependencies) so it can run
 * even if the ROS graph is in trouble and can be reused outside the node.
 *
 * - Records accumulate in a pre-allocated ring buffer and are written to a
 *   single append-only fd when the buffer fills.
 * - fsync is issued every `fsync_every` records (default 50) so at 50 Hz at
 *   most ~1 s of data is lost on power cut.
 * - The file is opened O_WRONLY|O_CREAT|O_APPEND and never seeks.
 *
 * Not thread safe: call from one thread (the node's executor thread).
 */
class FlightLogger {
  public:
    FlightLogger() = default;
    ~FlightLogger();

    FlightLogger(const FlightLogger &) = delete;
    FlightLogger &operator=(const FlightLogger &) = delete;

    /**
     * Open (or continue) the log file at `path`. Writes the file header if the
     * file is new/empty. Returns false on any I/O failure.
     */
    bool open(const std::string &path, size_t fsync_every = 50, size_t buffer_records = 64);

    bool isOpen() const { return fd >= 0; }

    /** Copy a record into the buffer; writes through when the buffer fills. */
    void log(const FlightLogRecord &record);

    /** Write any buffered records to the fd and fsync. */
    void flush();

    /** Flush, fsync, and close the fd. Safe to call repeatedly. */
    void close();

    uint64_t recordsLogged() const { return records_logged; }
    uint64_t writeFailures() const { return write_failures; }

  private:
    void writeBuffer();
    bool writeAll(const uint8_t *data, size_t len);

    int fd{-1};
    size_t fsync_every{50};
    size_t records_since_fsync{0};
    std::vector<FlightLogRecord> buffer; // pre-allocated; `pending` entries are valid
    size_t pending{0};
    uint64_t records_logged{0};
    uint64_t write_failures{0};
};

} // namespace cubesat_flight_logger
