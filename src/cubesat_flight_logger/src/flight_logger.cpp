#include "cubesat_flight_logger/flight_logger.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace cubesat_flight_logger {

FlightLogger::~FlightLogger() { close(); }

bool FlightLogger::open(const std::string &path, size_t fsync_every_in, size_t buffer_records) {
    close();

    fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0) {
        return false;
    }

    fsync_every = fsync_every_in > 0 ? fsync_every_in : 1;
    buffer.resize(buffer_records > 0 ? buffer_records : 1);
    pending = 0;
    records_since_fsync = 0;

    // only stamp the header on a brand-new file; with O_APPEND we never seek,
    // so re-opening an existing log just continues appending records
    struct stat st {};
    if (fstat(fd, &st) != 0) {
        close();
        return false;
    }
    if (st.st_size == 0) {
        FlightLogHeader header{};
        std::memcpy(header.magic, "ATLASLOG", 8);
        header.version = FLIGHT_LOG_VERSION;
        header.record_size = (uint16_t)sizeof(FlightLogRecord);
        if (!writeAll(reinterpret_cast<const uint8_t *>(&header), sizeof(header))) {
            close();
            return false;
        }
        ::fsync(fd);
    }
    return true;
}

void FlightLogger::log(const FlightLogRecord &record) {
    if (fd < 0) {
        return;
    }
    buffer[pending] = record;
    pending++;
    records_logged++;
    if (pending == buffer.size()) {
        writeBuffer();
    }
    if (records_since_fsync >= fsync_every) {
        ::fsync(fd);
        records_since_fsync = 0;
    }
}

void FlightLogger::writeBuffer() {
    if (fd < 0 || pending == 0) {
        return;
    }
    if (writeAll(reinterpret_cast<const uint8_t *>(buffer.data()), pending * sizeof(FlightLogRecord))) {
        records_since_fsync += pending;
    } else {
        write_failures += pending;
    }
    pending = 0;
}

bool FlightLogger::writeAll(const uint8_t *data, size_t len) {
    size_t written = 0;
    while (written < len) {
        ssize_t n = ::write(fd, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        written += (size_t)n;
    }
    return true;
}

void FlightLogger::flush() {
    if (fd < 0) {
        return;
    }
    writeBuffer();
    ::fsync(fd);
    records_since_fsync = 0;
}

void FlightLogger::close() {
    if (fd < 0) {
        return;
    }
    flush();
    ::close(fd);
    fd = -1;
}

} // namespace cubesat_flight_logger
