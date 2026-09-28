#pragma once

#include <Wire.h>
#include "remus/hal/IImu.hpp"

namespace remus::drivers {

enum class Mpu6050GyroRange : uint16_t {
  Dps500 = 500,
  Dps1000 = 1000,
  Dps2000 = 2000,
};

enum class Mpu6050AccelRange : uint8_t { G8 = 8, G16 = 16 };

class Mpu6050Imu final : public hal::IImu {
public:
  Mpu6050Imu(TwoWire& wire, int sda, int scl, uint16_t rateHz = 200,
             Mpu6050GyroRange gyroRange = Mpu6050GyroRange::Dps500,
             Mpu6050AccelRange accelRange = Mpu6050AccelRange::G8);

  bool begin() override;
  bool read(hal::ImuSample& sample) override;
  bool healthy() const override { return healthy_; }
  uint16_t sampleRateHz() const override { return rateHz_; }
  uint16_t gyroRangeDps() const { return static_cast<uint16_t>(gyroRange_); }
  uint8_t accelRangeG() const { return static_cast<uint8_t>(accelRange_); }
  uint8_t dlpfSetting() const { return 3; }
  const char* name() const override { return "MPU-6050"; }

private:
  bool configureAddress(uint8_t addr);

  TwoWire& wire_;
  int sda_;
  int scl_;
  uint16_t rateHz_;
  Mpu6050GyroRange gyroRange_;
  Mpu6050AccelRange accelRange_;
  uint8_t address_ = 0x68;
  bool healthy_ = false;
};

}  // namespace remus::drivers
