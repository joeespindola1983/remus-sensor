#pragma once

#include <cstdint>

namespace remus::orientation {

struct Vector3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quaternion {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

enum class Quality : uint8_t {
  Unavailable = 0,
  Stale = 1,
  Degraded = 2,
  Qualified = 3,
};

struct Sample {
  uint64_t nativeTimestampUs = 0;
  Vector3 accelerationG{};
  Vector3 rotationRateRadiansPerSecond{};
};

struct Estimate {
  Quaternion equipmentOrientationQuaternion{};
  Quality orientationQuality = Quality::Unavailable;
  uint64_t nativeTimestampUs = 0;
  bool mountingCalibrated = false;
};

struct Alignment {
  float relativeEquipmentAlignmentDegrees = 0.0f;
  Quality orientationQuality = Quality::Unavailable;
  uint64_t timeSeparationUs = 0;
  bool available = false;
};

class BladeOrientationEstimator {
 public:
  explicit BladeOrientationEstimator(float correctionGain = 1.5f);

  Estimate push(const Sample& sample);
  bool setMountingReference();
  void reset();
  Estimate estimate() const;

 private:
  float correctionGain_;
  Quaternion sensorOrientation_{};
  Quaternion mountingReferenceInverse_{};
  uint64_t lastTimestampUs_ = 0;
  uint32_t acceptedSamples_ = 0;
  Quality quality_ = Quality::Unavailable;
  bool initialized_ = false;
  bool mountingCalibrated_ = false;
};

Alignment relativeAlignment(const Estimate& left, const Estimate& right,
                            uint64_t maximumTimeSeparationUs = 20000);

}  // namespace remus::orientation
