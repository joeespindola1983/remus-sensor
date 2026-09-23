#include "remus/drivers/LinuxMpu6050.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace remus::drivers {

LinuxMpu6050::LinuxMpu6050(std::string device, uint8_t address, uint16_t rateHz)
    : device_(std::move(device)), address_(address), rateHz_(rateHz) {}

LinuxMpu6050::~LinuxMpu6050() {
  if (fd_ >= 0) ::close(fd_);
}

bool LinuxMpu6050::writeRegister(uint8_t reg, uint8_t value) {
  const uint8_t bytes[2] = {reg, value};
  const ssize_t written = ::write(fd_, bytes, sizeof(bytes));
  return written == static_cast<ssize_t>(sizeof(bytes));
}

bool LinuxMpu6050::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
  if (::write(fd_, &reg, 1) != 1) return false;
  return ::read(fd_, data, length) == static_cast<ssize_t>(length);
}

bool LinuxMpu6050::begin() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    std::cerr << "[IMU] Failed to open " << device_ << ": " << std::strerror(errno) << '\n';
    healthy_ = false;
    return false;
  }

  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "[IMU] Failed to select I2C address 0x" << std::hex << int(address_)
              << std::dec << ": " << std::strerror(errno) << '\n';
    healthy_ = false;
    return false;
  }

  // Match the ESP32 Prototype 1 sensor contract: +/-8g, +/-500 dps,
  // DLPF=3 and SMPLRT_DIV=4 -> 200 Hz from the MPU's 1 kHz base rate.
  healthy_ = writeRegister(0x6B, 0x00) &&
             writeRegister(0x1C, 0x10) &&
             writeRegister(0x1B, 0x08) &&
             writeRegister(0x1A, 0x03) &&
             writeRegister(0x19, 0x04);

  if (healthy_) {
    std::cout << "[IMU] OK " << name() << " @ " << device_ << " addr=0x"
              << std::hex << int(address_) << std::dec << " rate=" << rateHz_ << " Hz\n";
  } else {
    std::cerr << "[IMU] MPU-6050 configuration failed\n";
  }
  return healthy_;
}

bool LinuxMpu6050::read(hal::ImuSample& sample) {
  if (!healthy_) return false;

  uint8_t data[14]{};
  if (!readRegisters(0x3B, data, sizeof(data))) {
    healthy_ = false;
    return false;
  }

  auto be16 = [&](int offset) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(data[offset]) << 8) |
                                static_cast<uint16_t>(data[offset + 1]));
  };

  sample.rawAx = be16(0);
  sample.rawAy = be16(2);
  sample.rawAz = be16(4);
  sample.rawGx = be16(8);
  sample.rawGy = be16(10);
  sample.rawGz = be16(12);

  sample.accelX = sample.rawAx / 4096.0f;
  sample.accelY = sample.rawAy / 4096.0f;
  sample.accelZ = sample.rawAz / 4096.0f;
  sample.gyroX = sample.rawGx / 65.5f;
  sample.gyroY = sample.rawGy / 65.5f;
  sample.gyroZ = sample.rawGz / 65.5f;
  return true;
}

}  // namespace remus::drivers
