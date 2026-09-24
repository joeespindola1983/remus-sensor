#pragma once

#include "remus/Esp32HardwareProfile.hpp"

namespace remus::profiles {

// Development Remus Blade: ESP32-C3 + MPU-6050 + BLE.
// Product-owner wiring: SDA GPIO 6, SCL GPIO 5.
inline constexpr esp32::hw::HardwareProfile BladeDev {
  esp32::hw::ProfileId::BladeDev,
  "BLDDEV",
  "REMUS Blade Development / ESP32-C3",
  esp32::hw::ImuModel::Mpu6050,
  esp32::hw::GpsModel::None,
  esp32::hw::StorageModel::None,
  esp32::hw::DisplayModel::None,
  false,
  false,
  false,
  true,
  true,
  {6, 5},
  {-1, -1},
  {-1, -1, -1, -1},
  {-1, -1, -1, -1, -1},
  200
};

static_assert(esp32::hw::profileIsValid(BladeDev),
              "REMUS Blade development profile has an invalid GPIO map");
static_assert(esp32::hw::capabilityByte(BladeDev) == 0x0C,
              "REMUS Blade development capabilities changed unexpectedly");

}  // namespace remus::profiles
