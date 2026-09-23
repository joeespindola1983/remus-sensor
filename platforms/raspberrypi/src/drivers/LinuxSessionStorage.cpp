#include "remus/drivers/LinuxSessionStorage.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

namespace remus::drivers {

LinuxSessionStorage::LinuxSessionStorage(std::string directory)
    : directory_(std::move(directory)) {}

LinuxSessionStorage::~LinuxSessionStorage() {
  stopSession();
}

bool LinuxSessionStorage::begin() {
  std::error_code ec;
  std::filesystem::create_directories(directory_, ec);
  if (ec) {
    std::cerr << "[STORAGE] Cannot create " << directory_ << ": " << ec.message() << '\n';
    healthy_ = false;
    return false;
  }
  healthy_ = true;
  std::cout << "[STORAGE] OK " << name() << " -> " << std::filesystem::absolute(directory_) << '\n';
  return true;
}

uint32_t LinuxSessionStorage::randomSessionId() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist;
  return dist(gen);
}

bool LinuxSessionStorage::startSession(uint32_t startedAtMs, uint16_t imuRateHz, uint32_t profileTag, bool hasGps) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!healthy_) return false;
  if (stream_.is_open()) return true;

  const uint32_t id = randomSessionId();
  std::ostringstream name;
  name << "remus_sensor_" << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << id << ".bin";
  currentPath_ = (std::filesystem::path(directory_) / name.str()).string();

  stream_.open(currentPath_, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!stream_) {
    std::cerr << "[STORAGE] Failed to create " << currentPath_ << '\n';
    healthy_ = false;
    return false;
  }

  auto header = session::makeHeader(startedAtMs, id, imuRateHz);
  // RBP1 padding was already reserved. Tag the producer without changing the
  // 32-byte contract: byte 0 = profile (2), byte 1 = IMU model (1/MPU6050),
  // byte 2 = GPS model (1/NEO6M or 0/None), byte 3 = capabilities (0x13 with GPS, 0x11 without).
  header.padding[0] = static_cast<uint8_t>(profileTag);
  header.padding[1] = 1;
  header.padding[2] = hasGps ? 1 : 0;
  header.padding[3] = hasGps ? 0x13 : 0x11;
  stream_.write(reinterpret_cast<const char*>(&header), sizeof(header));
  stream_.flush();
  recordsWritten_.store(0);
  std::cout << "[STORAGE] RECORDING -> " << currentPath_ << '\n';
  return static_cast<bool>(stream_);
}

bool LinuxSessionStorage::writeImu(const session::ImuRecord& record) { return writeRecord(record); }
bool LinuxSessionStorage::writeGps(const session::GpsRecord& record) { return writeRecord(record); }
bool LinuxSessionStorage::writeSpm(const session::SpmRecord& record) { return writeRecord(record); }

void LinuxSessionStorage::flush() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_.is_open()) stream_.flush();
}

void LinuxSessionStorage::stopSession() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) return;
  stream_.flush();
  stream_.close();
  std::cout << "[STORAGE] STOPPED -> " << currentPath_ << " records=" << recordsWritten_.load() << '\n';
}

bool LinuxSessionStorage::recording() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_.is_open();
}

}  // namespace remus::drivers
