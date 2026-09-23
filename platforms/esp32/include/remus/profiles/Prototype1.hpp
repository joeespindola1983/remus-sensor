#pragma once

#include "remus/Esp32HardwareProfile.hpp"

namespace remus::profiles {

// Prototype 1: ESP32-C3 + MPU-6050 + NEO-6M + MicroSD + GMT024-10 V2.1.
// Wiring confirmed on the working ESP32-C3 Prototype 1 bench.
inline constexpr esp32::hw::HardwareProfile Prototype1 {
  esp32::hw::ProfileId::Prototype1,
  "P1",
  "REMUS Prototype 1 / ESP32-C3",
  esp32::hw::ImuModel::Mpu6050,
  esp32::hw::GpsModel::Neo6m,
  esp32::hw::StorageModel::MicroSdSpi,
  esp32::hw::DisplayModel::Gmt024St7789,
  true,
  true,
  true,
  true,
  true,
  {5, 6},           // MPU SDA/SCL
  {0, 1},           // GPS RX/TX from ESP32 perspective
  {8, 10, 20, 21},  // SD MISO/MOSI/SCK/CS
  {4, 2, 7, 3, 9},  // TFT SCK/MOSI/DC/RST/CS (software SPI)
  200
};

static_assert(esp32::hw::profileIsValid(Prototype1),
              "REMUS Prototype 1 has an invalid/conflicting GPIO map");

}  // namespace remus::profiles
