#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

#include "remus/core/SessionFormat.hpp"
#include "remus/hal/IStorage.hpp"

namespace remus::drivers {

class LinuxSessionStorage final : public hal::IStorage {
public:
  explicit LinuxSessionStorage(std::string directory);
  ~LinuxSessionStorage() override;

  bool begin() override;
  bool healthy() const override { return healthy_; }
  const char* name() const override { return "Linux filesystem"; }

  bool startSession(uint32_t startedAtMs, uint16_t imuRateHz, uint32_t profileTag = 2, bool hasGps = true);
  bool writeImu(const session::ImuRecord& record);
  bool writeGps(const session::GpsRecord& record);
  bool writeSpm(const session::SpmRecord& record);
  void flush();
  void stopSession();

  bool recording() const;
  const std::string& currentPath() const { return currentPath_; }
  uint32_t recordsWritten() const { return recordsWritten_.load(); }

private:
  template <typename T>
  bool writeRecord(const T& record) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open()) return false;
    stream_.write(reinterpret_cast<const char*>(&record), sizeof(record));
    if (!stream_) {
      healthy_ = false;
      return false;
    }
    recordsWritten_.fetch_add(1);
    return true;
  }

  static uint32_t randomSessionId();

  std::string directory_;
  std::string currentPath_;
  mutable std::mutex mutex_;
  std::ofstream stream_;
  bool healthy_ = false;
  std::atomic<uint32_t> recordsWritten_{0};
};

}  // namespace remus::drivers
