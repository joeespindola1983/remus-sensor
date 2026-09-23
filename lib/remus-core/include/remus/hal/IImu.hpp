#pragma once

#include <cstdint>

namespace remus::hal {

struct ImuSample {
  int16_t rawAx = 0, rawAy = 0, rawAz = 0;
  int16_t rawGx = 0, rawGy = 0, rawGz = 0;

  // Canonical driver output used by existing REMUS code.
  float accelX = 0, accelY = 0, accelZ = 0; // g
  float gyroX = 0, gyroY = 0, gyroZ = 0;    // deg/s
};

class IImu {
public:
  virtual ~IImu() = default;
  virtual bool begin() = 0;
  virtual bool read(ImuSample& sample) = 0;
  virtual bool healthy() const = 0;
  virtual uint16_t sampleRateHz() const = 0;
  virtual const char* name() const = 0;
};

}  // namespace remus::hal
