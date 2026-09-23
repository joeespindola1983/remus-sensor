#pragma once

#include <cstdint>

namespace remus::profiles {

struct RaspberryPiProfile {
  const char* code;
  const char* displayName;
  const char* i2cDevice;
  uint8_t imuAddress;
  uint16_t imuRateHz;
  const char* gpsDevice;
  uint32_t gpsBaud;
  bool hasOled;
  uint8_t oledAddress;
  const char* defaultSessionDir;
};

// Prototype 2: Raspberry Pi Zero W v1.1 + MPU-6050 + NEO-6M + SSD1306.
inline constexpr RaspberryPiProfile Prototype2 {
  "P2",
  "REMUS Prototype 2 / Raspberry Pi Zero W v1.1",
  "/dev/i2c-1",
  0x68,
  200,
  "/dev/serial0",
  9600,
  true,
  0x3C,
  "./sessions"
};

}  // namespace remus::profiles
