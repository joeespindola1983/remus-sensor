#pragma once

#include <cstdint>
#include <string>

#include "remus/hal/IImu.hpp"

namespace remus::drivers {

class LinuxMpu6050 final : public hal::IImu {
public:
  LinuxMpu6050(std::string device, uint8_t address, uint16_t rateHz = 200);
  ~LinuxMpu6050() override;

  bool begin() override;
  bool read(hal::ImuSample& sample) override;
  bool healthy() const override { return healthy_; }
  uint16_t sampleRateHz() const override { return rateHz_; }
  const char* name() const override { return "MPU-6050/Linux I2C"; }

private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* data, size_t length);

  std::string device_;
  uint8_t address_;
  uint16_t rateHz_;
  int fd_ = -1;
  bool healthy_ = false;
};

}  // namespace remus::drivers
