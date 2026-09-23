#include "remus/drivers/LinuxNeo6m.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace remus::drivers {
namespace {

speed_t baudFlag(uint32_t baud) {
  switch (baud) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B9600;
  }
}

std::vector<std::string> splitFields(const std::string& line) {
  std::vector<std::string> fields;
  size_t start = 0;
  size_t end = line.find('*');
  if (end == std::string::npos) end = line.size();
  while (start <= end) {
    const size_t comma = line.find(',', start);
    const size_t stop = (comma == std::string::npos || comma > end) ? end : comma;
    fields.emplace_back(line.substr(start, stop - start));
    if (stop == end) break;
    start = stop + 1;
  }
  return fields;
}

}  // namespace

LinuxNeo6m::LinuxNeo6m(std::string device, uint32_t baud)
    : device_(std::move(device)), baud_(baud) {}

LinuxNeo6m::~LinuxNeo6m() {
  if (fd_ >= 0) ::close(fd_);
}

bool LinuxNeo6m::begin() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    std::cerr << "[GPS] Failed to open " << device_ << ": " << std::strerror(errno) << '\n';
    healthy_ = false;
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[GPS] tcgetattr failed: " << std::strerror(errno) << '\n';
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = baudFlag(baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[GPS] tcsetattr failed: " << std::strerror(errno) << '\n';
    return false;
  }

  tcflush(fd_, TCIFLUSH);
  healthy_ = true;
  std::cout << "[GPS] OK " << name() << " @ " << device_ << " " << baud_ << " baud\n";
  return true;
}

std::chrono::milliseconds LinuxNeo6m::age(std::chrono::steady_clock::time_point point) {
  if (point.time_since_epoch().count() == 0) return std::chrono::milliseconds::max();
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - point);
}

bool LinuxNeo6m::fixValid() const {
  return fixValidFlag_ && age(lastFix_).count() < 2500;
}

bool LinuxNeo6m::hdopValidRecent() const {
  return age(lastHdop_).count() < 2500;
}

bool LinuxNeo6m::locationUpdated() const {
  const bool value = locationUpdated_;
  locationUpdated_ = false;
  return value;
}

int LinuxNeo6m::toInt(const std::string& value, int fallback) {
  try { return value.empty() ? fallback : std::stoi(value); }
  catch (...) { return fallback; }
}

double LinuxNeo6m::toDouble(const std::string& value, double fallback) {
  try { return value.empty() ? fallback : std::stod(value); }
  catch (...) { return fallback; }
}

std::string LinuxNeo6m::field(const std::string& line, size_t index) {
  const auto fields = splitFields(line);
  return index < fields.size() ? fields[index] : std::string{};
}

bool LinuxNeo6m::checksumOk(const std::string& line) {
  if (line.empty() || line[0] != '$') return false;
  const size_t star = line.find('*');
  if (star == std::string::npos || star + 2 >= line.size()) return true; // tolerate checksum-less test streams
  uint8_t sum = 0;
  for (size_t i = 1; i < star; ++i) sum ^= static_cast<uint8_t>(line[i]);
  unsigned expected = 0;
  std::istringstream ss(line.substr(star + 1, 2));
  ss >> std::hex >> expected;
  return sum == static_cast<uint8_t>(expected);
}

double LinuxNeo6m::parseCoordinate(const std::string& value, const std::string& hemi) {
  if (value.empty()) return 0.0;
  const double raw = toDouble(value, 0.0);
  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double result = degrees + minutes / 60.0;
  if (hemi == "S" || hemi == "W") result = -result;
  return result;
}

void LinuxNeo6m::parseRmc(const std::string& line) {
  const auto f = splitFields(line);
  if (f.size() < 10) return;

  if (f[1].size() >= 6) {
    hour_ = static_cast<uint8_t>(toInt(f[1].substr(0, 2)));
    minute_ = static_cast<uint8_t>(toInt(f[1].substr(2, 2)));
    second_ = static_cast<uint8_t>(toInt(f[1].substr(4, 2)));
    timeValid_ = true;
  }

  const bool active = f[2] == "A";
  if (active && !f[3].empty() && !f[5].empty()) {
    latitude_ = parseCoordinate(f[3], f[4]);
    longitude_ = parseCoordinate(f[5], f[6]);
    speedKmph_ = static_cast<float>(toDouble(f[7]) * 1.852); // knots -> km/h
    fixValidFlag_ = true;
    lastFix_ = std::chrono::steady_clock::now();
    locationUpdated_ = true;
  } else if (!active) {
    fixValidFlag_ = false;
  }

  if (f[9].size() == 6) {
    day_ = static_cast<uint8_t>(toInt(f[9].substr(0, 2)));
    month_ = static_cast<uint8_t>(toInt(f[9].substr(2, 2)));
    const int yy = toInt(f[9].substr(4, 2));
    year_ = static_cast<uint16_t>(yy >= 80 ? 1900 + yy : 2000 + yy);
    dateValid_ = true;
  }
}

void LinuxNeo6m::parseGga(const std::string& line) {
  const auto f = splitFields(line);
  if (f.size() < 9) return;

  if (f[1].size() >= 6) {
    hour_ = static_cast<uint8_t>(toInt(f[1].substr(0, 2)));
    minute_ = static_cast<uint8_t>(toInt(f[1].substr(2, 2)));
    second_ = static_cast<uint8_t>(toInt(f[1].substr(4, 2)));
    timeValid_ = true;
  }

  const int quality = toInt(f[6]);
  satellitesInUse_ = static_cast<uint8_t>(std::clamp(toInt(f[7]), 0, 255));
  if (!f[8].empty()) {
    hdop_ = static_cast<float>(toDouble(f[8]));
    lastHdop_ = std::chrono::steady_clock::now();
  }

  if (quality > 0 && !f[2].empty() && !f[4].empty()) {
    latitude_ = parseCoordinate(f[2], f[3]);
    longitude_ = parseCoordinate(f[4], f[5]);
    fixValidFlag_ = true;
    lastFix_ = std::chrono::steady_clock::now();
    locationUpdated_ = true;
  }
}

void LinuxNeo6m::parseGsv(const std::string& line) {
  const auto f = splitFields(line);
  if (f.size() < 4) return;
  satellitesInView_ = std::max(satellitesInView_, toInt(f[3]));
  int sentenceMax = 0;
  for (size_t i = 7; i < f.size(); i += 4) sentenceMax = std::max(sentenceMax, toInt(f[i]));
  maxSnr_ = std::max(maxSnr_, sentenceMax);
  // First GSV sentence starts a new reporting cycle.
  if (toInt(f[2]) == 1) maxSnr_ = sentenceMax;
}

void LinuxNeo6m::processLine(const std::string& line, bool showRawNmea, bool workoutActive) {
  if (showRawNmea && !workoutActive) std::cout << line << '\n';
  if (!checksumOk(line)) return;
  const std::string type = field(line, 0);
  if (type.size() < 3) return;
  const std::string suffix = type.substr(type.size() - 3);
  if (suffix == "RMC") parseRmc(line);
  else if (suffix == "GGA") parseGga(line);
  else if (suffix == "GSV") parseGsv(line);
}

void LinuxNeo6m::poll(bool showRawNmea, bool workoutActive) {
  if (!healthy_ || fd_ < 0) return;
  char buf[512];
  while (true) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      charsProcessed_ += static_cast<uint64_t>(n);
      for (ssize_t i = 0; i < n; ++i) {
        const char c = buf[i];
        if (c == '\n' || c == '\r') {
          if (!lineBuffer_.empty()) {
            processLine(lineBuffer_, showRawNmea, workoutActive);
            lineBuffer_.clear();
          }
        } else if (lineBuffer_.size() < 180) {
          lineBuffer_.push_back(c);
        } else {
          lineBuffer_.clear();
        }
      }
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "[GPS] read failed: " << std::strerror(errno) << '\n';
      healthy_ = false;
    }
    break;
  }
}

void LinuxNeo6m::aidPosition(float, float) {
  // NEO-6M aiding is deliberately not emulated with PMTK/PCAS commands here.
  // Keep the HAL method so the application protocol can stay platform-neutral.
}

}  // namespace remus::drivers
