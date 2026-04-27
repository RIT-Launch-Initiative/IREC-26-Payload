#include "nmea_listener/NmeaListenerNode.hpp"
#include "nmea/message/rmc.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <nmea/message/gga.hpp>
#include <nmea/sentence.hpp>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

static speed_t baudToSpeed(int baud) {
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

NmeaListenerNode::NmeaListenerNode(const rclcpp::NodeOptions &options)
    : Node("nmea_listener", options) {
  port = declare_parameter<std::string>("port", "/dev/ttyS0"); // methinks this is the default for pi zero 2w
  baud = declare_parameter<int>("baud", 9600);
  gpsStatusTopic = declare_parameter<std::string>("gps_status_topic", "/gps/status");
  maxLineLen = declare_parameter<int>("max_line_len", 512);

  gpsStatusPublisher = create_publisher<arm_msgs::msg::GpsStatus>(gpsStatusTopic, rclcpp::QoS(10));

  fd = openSerial(port, baud);
  RCLCPP_INFO(get_logger(), "Opened %s @ %d baud", port.c_str(), baud);

  running.store(true);
  readerThread = std::thread(&NmeaListenerNode::readerLoop, this);

  timer = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&NmeaListenerNode::publishGpsStatusCallback, this)
  );

  RCLCPP_INFO(get_logger(), "NMEA Listener Node started.");
}

NmeaListenerNode::~NmeaListenerNode() {
  running.store(false);

  if (readerThread.joinable()) {
    readerThread.join();
  }

  if (fd >= 0) {
    ::close(fd);
  }
}

int NmeaListenerNode::openSerial(const std::string &device, int baudRate) {
  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("open failed: " +
                             std::string(std::strerror(errno)));
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error("tcgetattr failed");
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

  speed_t speed = baudToSpeed(baudRate);
  if (speed == 0) {
    ::close(fd);
    throw std::runtime_error("Unsupported baud rate");
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error("tcsetattr failed");
  }

  tcflush(fd, TCIOFLUSH);
  return fd;
}

void NmeaListenerNode::readerLoop() {
  std::vector<char> readBuf(4096);
  std::string buffer;
  buffer.reserve(4096);

  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;

  while (running.load()) {
    int ret = ::poll(&pfd, 1, 200);
    if (ret <= 0) {
      if (ret < 0 && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "poll error: %s", std::strerror(errno));
      }
      continue;
    }

    if (!(pfd.revents & POLLIN)) {
      continue;
    }

    ssize_t n = ::read(fd, readBuf.data(), readBuf.size());
    if (n <= 0) {
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "read error: %s", std::strerror(errno));
      }
      continue;
    }

    buffer.append(readBuf.data(), static_cast<size_t>(n));

    while (true) {
      size_t start = buffer.find('$');
      if (start == std::string::npos) {
        buffer.clear();
        break;
      }

      if (start > 0) {
        buffer.erase(0, start);
      }

      size_t lf = buffer.find('\n');
      if (lf == std::string::npos) {
        break;
      }

      std::string line = buffer.substr(0, lf + 1);
      buffer.erase(0, lf + 1);

      while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
        line.pop_back();
      }

      if (line.empty()) {
        continue;
      }

      if (static_cast<int>(line.size()) > maxLineLen) {
        RCLCPP_WARN(get_logger(), "Dropping overlong NMEA line (%zu bytes)",
                    line.size());
        continue;
      }

      handleRawNmeaLine(line);
    }

    if (buffer.size() > static_cast<size_t>(maxLineLen * 4)) {
      RCLCPP_WARN(get_logger(),
                  "UART buffer overflow without newline, clearing");
      buffer.clear();
    }
  }
}


void NmeaListenerNode::handleRawNmeaLine(const std::string &line) {
  nmea::sentence sentence(line);
  RCLCPP_DEBUG(this->get_logger(), "Received NMEA sentence: %s", line.c_str());

  static constexpr std::array<std::string_view, 5> ignored_sentences = {
      "GSA", "GSV", "GLL", "RMC", "VTG"
  };
  
  if (sentence.type() == "GGA") {
    nmea::gga gga(sentence);
    std::lock_guard<std::mutex> lock(currentStateMutex);
    currentState.latitude = gga.latitude.get();
    currentState.longitude = gga.longitude.get();
    currentState.fix_type = static_cast<uint8_t>(gga.fix.get());
    currentState.satellites_visible = gga.satellite_count.get();
    currentState.timestamp = static_cast<uint32_t>(gga.utc.get());
    RCLCPP_DEBUG(this->get_logger(), "GGA parsed: lat=%f, lon=%f, fix=%d, sats=%d, time=%u",
                  currentState.latitude,
                  currentState.longitude,
                  currentState.fix_type,
                  currentState.satellites_visible,
                  currentState.timestamp);
  } else if (std::find(ignored_sentences.begin(), ignored_sentences.end(), sentence.type()) != ignored_sentences.end()) {
    RCLCPP_DEBUG(this->get_logger(), "Ignoring NMEA sentence type: %s", sentence.type().c_str());
    return;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unhandled NMEA sentence type: %s", sentence.type().c_str());
  }

  // TODO: Should we do RMC?
}

void NmeaListenerNode::publishGpsStatusCallback() {
  arm_msgs::msg::GpsStatus msg;

  {
    std::lock_guard<std::mutex> lock(currentStateMutex);
    msg.latitude = currentState.latitude;
    msg.longitude = currentState.longitude;
    msg.fix_type = currentState.fix_type;
    msg.satellites_visible = currentState.satellites_visible;
    msg.timestamp = currentState.timestamp;
  }

  gpsStatusPublisher->publish(msg);
}