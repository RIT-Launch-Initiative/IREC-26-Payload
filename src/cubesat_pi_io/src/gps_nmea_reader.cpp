#include "cubesat_pi_io/gps_nmea_reader.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

#include <nmea/message/gga.hpp>
#include <nmea/sentence.hpp>

namespace cubesat_pi_io {

namespace {

speed_t baudToSpeed(int baud) {
    switch (baud) {
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    default:
        return 0;
    }
}

} // namespace

GpsNmeaReader::~GpsNmeaReader() { close(); }

void GpsNmeaReader::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
    buffer.clear();
}

bool GpsNmeaReader::open(const std::string &device, int baud_rate) {
    close();

    fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close();
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed = baudToSpeed(baud_rate);
    if (speed == 0) {
        close();
        return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close();
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    buffer.reserve(kReadChunk);
    return true;
}

std::optional<GpsFix> GpsNmeaReader::readFix() {
    if (fd < 0) {
        return std::nullopt;
    }
    drainAndParse();
    if (!have_fix) {
        return std::nullopt;
    }
    return fix;
}

void GpsNmeaReader::drainAndParse() {
    std::vector<char> chunk(kReadChunk);

    while (true) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int ret = ::poll(&pfd, 1, 0);
        if (ret <= 0 || !(pfd.revents & POLLIN)) {
            break;
        }

        ssize_t n = ::read(fd, chunk.data(), chunk.size());
        if (n <= 0) {
            break;
        }

        buffer.append(chunk.data(), static_cast<std::size_t>(n));

        while (true) {
            auto start = buffer.find('$');
            if (start == std::string::npos) {
                buffer.clear();
                break;
            }
            if (start > 0) {
                buffer.erase(0, start);
            }

            auto lf = buffer.find('\n');
            if (lf == std::string::npos) {
                break;
            }

            std::string line = buffer.substr(0, lf + 1);
            buffer.erase(0, lf + 1);

            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
                line.pop_back();
            }
            if (line.empty() || line.size() > kMaxLineLen) {
                continue;
            }

            handleLine(line);
        }

        // Guard against runaway buffer growth (no newlines arriving).
        if (buffer.size() > kMaxLineLen * 4) {
            buffer.clear();
        }
    }
}

void GpsNmeaReader::handleLine(const std::string &line) {
    nmea::sentence sentence(line);
    if (sentence.type() != "GGA") {
        return;
    }

    nmea::gga gga(sentence);

    fix.latitude = gga.latitude.exists() ? gga.latitude.get() : 0.0;
    fix.longitude = gga.longitude.exists() ? gga.longitude.get() : 0.0;
    fix.altitude_m = gga.altitude.exists() ? gga.altitude.get() : 0.0f;
    fix.fix_type = gga.fix.exists() ? static_cast<uint8_t>(gga.fix.get()) : 0;
    fix.satellites_visible = gga.satellite_count.exists() ? gga.satellite_count.get() : 0;
    fix.gps_time = gga.utc.exists() ? static_cast<uint32_t>(gga.utc.get()) : 0;
    fix.valid = fix.fix_type != 0;
    have_fix = true;
}

} // namespace cubesat_pi_io
