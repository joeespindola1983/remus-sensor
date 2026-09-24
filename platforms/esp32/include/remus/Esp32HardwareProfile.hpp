#pragma once

#include <cstdint>

namespace remus::esp32::hw {

enum class ProfileId : uint8_t { Prototype1 = 1, BladeDev = 2 };
enum class ImuModel : uint8_t { Mpu6050 = 1 };
enum class GpsModel : uint8_t { None = 0, Neo6m = 1 };
enum class StorageModel : uint8_t { None = 0, MicroSdSpi = 1 };
enum class DisplayModel : uint8_t { None = 0, Gmt024St7789 = 1 };

struct I2cPins { int8_t sda = -1; int8_t scl = -1; };
struct UartPins { int8_t rx = -1; int8_t tx = -1; };
struct SpiPins { int8_t miso = -1; int8_t mosi = -1; int8_t sck = -1; int8_t cs = -1; };
struct DisplayPins { int8_t sck = -1; int8_t mosi = -1; int8_t dc = -1; int8_t rst = -1; int8_t cs = -1; };

struct HardwareProfile {
  ProfileId id;
  const char* code;
  const char* displayName;
  ImuModel imuModel;
  GpsModel gpsModel;
  StorageModel storageModel;
  DisplayModel displayModel;
  bool hasGps;
  bool hasStorage;
  bool hasDisplay;
  bool hasBle;
  bool hasUsbRecovery;
  I2cPins imuPins;
  UartPins gpsPins;
  SpiPins sdPins;
  DisplayPins displayPins;
  uint16_t imuRateHz;
};

constexpr bool assigned(int pin) { return pin >= 0; }

constexpr bool hasDuplicatePins(const HardwareProfile& p) {
  const int pins[] = {
    p.imuPins.sda, p.imuPins.scl,
    p.hasGps ? p.gpsPins.rx : -1,
    p.hasGps ? p.gpsPins.tx : -1,
    p.hasStorage ? p.sdPins.miso : -1,
    p.hasStorage ? p.sdPins.mosi : -1,
    p.hasStorage ? p.sdPins.sck : -1,
    p.hasStorage ? p.sdPins.cs : -1,
    p.hasDisplay ? p.displayPins.sck : -1,
    p.hasDisplay ? p.displayPins.mosi : -1,
    p.hasDisplay ? p.displayPins.dc : -1,
    p.hasDisplay ? p.displayPins.rst : -1,
    p.hasDisplay ? p.displayPins.cs : -1,
  };
  for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    if (!assigned(pins[i])) continue;
    for (unsigned j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
      if (pins[i] == pins[j]) return true;
    }
  }
  return false;
}

constexpr uint8_t capabilityByte(const HardwareProfile& p) {
  return (p.hasGps ? 0x01 : 0x00) |
         (p.hasStorage ? 0x02 : 0x00) |
         (p.hasBle ? 0x04 : 0x00) |
         (p.hasUsbRecovery ? 0x08 : 0x00) |
         (p.hasDisplay ? 0x10 : 0x00);
}

constexpr bool profileIsValid(const HardwareProfile& p) {
  if (!assigned(p.imuPins.sda) || !assigned(p.imuPins.scl) || p.imuRateHz == 0) return false;
  if (p.hasGps && (!assigned(p.gpsPins.rx) || !assigned(p.gpsPins.tx))) return false;
  if (p.hasStorage && (!assigned(p.sdPins.miso) || !assigned(p.sdPins.mosi) ||
                       !assigned(p.sdPins.sck) || !assigned(p.sdPins.cs))) return false;
  if (p.hasDisplay && (!assigned(p.displayPins.sck) || !assigned(p.displayPins.mosi) ||
                       !assigned(p.displayPins.dc) || !assigned(p.displayPins.rst) ||
                       !assigned(p.displayPins.cs))) return false;
  return !hasDuplicatePins(p);
}

}  // namespace remus::esp32::hw
