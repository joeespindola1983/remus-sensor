#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "remus/hal/IGps.hpp"

namespace remus::drivers {

class LinuxNeo6m final : public hal::IGps {
public:
  LinuxNeo6m(std::string device, uint32_t baud = 9600);
  ~LinuxNeo6m() override;

  bool begin() override;
  void poll(bool showRawNmea, bool workoutActive) override;
  bool healthy() const override { return healthy_; }
  bool fixValid() const override;
  bool locationUpdated() const override;
  double latitude() const override { return latitude_; }
  double longitude() const override { return longitude_; }
  float speedKmph() const override { return speedKmph_; }
  uint8_t satellitesInUse() const override { return satellitesInUse_; }
  int satellitesInView() const override { return satellitesInView_; }
  int maxSnr() const override { return maxSnr_; }
  bool hdopValidRecent() const override;
  float hdop() const override { return hdop_; }
  uint64_t charsProcessed() const override { return charsProcessed_; }
  bool timeValid() const override { return timeValid_; }
  uint8_t hour() const override { return hour_; }
  uint8_t minute() const override { return minute_; }
  uint8_t second() const override { return second_; }
  bool dateValid() const override { return dateValid_; }
  uint8_t day() const override { return day_; }
  uint8_t month() const override { return month_; }
  uint16_t year() const override { return year_; }
  void aidPosition(float lat, float lon) override;
  const char* name() const override { return "NEO-6M/Linux UART"; }

private:
  void processLine(const std::string& line, bool showRawNmea, bool workoutActive);
  void parseRmc(const std::string& line);
  void parseGga(const std::string& line);
  void parseGsv(const std::string& line);
  static bool checksumOk(const std::string& line);
  static double parseCoordinate(const std::string& value, const std::string& hemi);
  static std::string field(const std::string& line, size_t index);
  static int toInt(const std::string& value, int fallback = 0);
  static double toDouble(const std::string& value, double fallback = 0.0);
  static std::chrono::milliseconds age(std::chrono::steady_clock::time_point point);

  std::string device_;
  uint32_t baud_;
  int fd_ = -1;
  bool healthy_ = false;
  std::string lineBuffer_;

  double latitude_ = 0.0;
  double longitude_ = 0.0;
  float speedKmph_ = 0.0f;
  uint8_t satellitesInUse_ = 0;
  int satellitesInView_ = 0;
  int maxSnr_ = 0;
  float hdop_ = 0.0f;
  uint64_t charsProcessed_ = 0;

  mutable bool locationUpdated_ = false;
  bool fixValidFlag_ = false;
  std::chrono::steady_clock::time_point lastFix_{};
  std::chrono::steady_clock::time_point lastHdop_{};

  bool timeValid_ = false;
  uint8_t hour_ = 0, minute_ = 0, second_ = 0;
  bool dateValid_ = false;
  uint8_t day_ = 0, month_ = 0;
  uint16_t year_ = 0;
};

}  // namespace remus::drivers
