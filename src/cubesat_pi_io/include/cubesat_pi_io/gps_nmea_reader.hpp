#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace cubesat_pi_io {

struct GpsFix {
  bool valid;
  double latitude;
  double longitude;
  float altitude_m;
  uint8_t fix_type;
  uint8_t satellites_visible;
  uint32_t gps_time;
};

class GpsNmeaReader {
public:
  GpsNmeaReader() = default;
  ~GpsNmeaReader();

  GpsNmeaReader(const GpsNmeaReader &) = delete;
  GpsNmeaReader &operator=(const GpsNmeaReader &) = delete;

  bool open(const std::string &device, int baud_rate);
  void close();
  bool isOpen() const { return fd >= 0; }

  // Drains pending UART bytes, parses any complete NMEA sentences, and
  // updates the cached fix. Returns the cached fix if at least one GGA
  // sentence has ever been parsed; otherwise std::nullopt.
  std::optional<GpsFix> readFix();

private:
  void drainAndParse();
  void handleLine(const std::string &line);

  int fd{-1};
  std::string buffer;

  GpsFix fix{};
  bool have_fix{false};

  static constexpr std::size_t kMaxLineLen = 512;
  static constexpr std::size_t kReadChunk = 4096;
};

} // namespace cubesat_pi_io
