#pragma once

#include <Wire.h>
#include "remus/hal/IImu.hpp"

namespace remus::drivers {

class Mpu6050Imu final : public hal::IImu {
public:
  Mpu6050Imu(TwoWire& wire, int sda, int scl, uint16_t rateHz = 200);

  bool begin() override;
  bool read(hal::ImuSample& sample) override;
  bool healthy() const override { return healthy_; }
  uint16_t sampleRateHz() const override { return rateHz_; }
  const char* name() const override { return "MPU-6050"; }

private:
  bool configureAddress(uint8_t addr);

  TwoWire& wire_;
  int sda_;
  int scl_;
  uint16_t rateHz_;
  uint8_t address_ = 0x68;
  bool healthy_ = false;
};

}  // namespace remus::drivers
