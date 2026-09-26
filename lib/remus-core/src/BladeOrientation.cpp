#include "remus/core/BladeOrientation.hpp"

#include <algorithm>
#include <cmath>

namespace remus::orientation {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinimumAccelNormG = 0.70f;
constexpr float kMaximumAccelNormG = 1.30f;
constexpr uint64_t kMaximumIntegrationGapUs = 50000;
constexpr uint32_t kQualificationSamples = 25;

float norm(Vector3 value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vector3 normalized(Vector3 value) {
  const float length = norm(value);
  if (length <= 1.0e-6f) return {};
  return {value.x / length, value.y / length, value.z / length};
}

Quaternion normalized(Quaternion value) {
  const float length = std::sqrt(value.w * value.w + value.x * value.x +
                                 value.y * value.y + value.z * value.z);
  if (length <= 1.0e-6f) return {};
  return {value.w / length, value.x / length, value.y / length, value.z / length};
}

Quaternion multiply(Quaternion left, Quaternion right) {
  return {
      left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
      left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
      left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
      left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
  };
}

Quaternion inverse(Quaternion value) {
  value = normalized(value);
  return {value.w, -value.x, -value.y, -value.z};
}

Vector3 estimatedGravity(Quaternion q) {
  q = normalized(q);
  return {
      2.0f * (q.x * q.z - q.w * q.y),
      2.0f * (q.w * q.x + q.y * q.z),
      q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
  };
}

Vector3 cross(Vector3 left, Vector3 right) {
  return {
      left.y * right.z - left.z * right.y,
      left.z * right.x - left.x * right.z,
      left.x * right.y - left.y * right.x,
  };
}

Quaternion fromGravity(Vector3 gravity) {
  gravity = normalized(gravity);
  const float roll = std::atan2(gravity.y, gravity.z);
  const float pitch = std::atan2(-gravity.x,
                                 std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
  const float cr = std::cos(roll * 0.5f);
  const float sr = std::sin(roll * 0.5f);
  const float cp = std::cos(pitch * 0.5f);
  const float sp = std::sin(pitch * 0.5f);
  return normalized({cr * cp, sr * cp, cr * sp, -sr * sp});
}

Quaternion integrate(Quaternion q, Vector3 omega, float dt) {
  const Quaternion derivative = multiply(q, {0.0f, omega.x, omega.y, omega.z});
  q.w += 0.5f * derivative.w * dt;
  q.x += 0.5f * derivative.x * dt;
  q.y += 0.5f * derivative.y * dt;
  q.z += 0.5f * derivative.z * dt;
  return normalized(q);
}

Quality lowerQuality(Quality left, Quality right) {
  return static_cast<uint8_t>(left) < static_cast<uint8_t>(right) ? left : right;
}

}  // namespace

BladeOrientationEstimator::BladeOrientationEstimator(float correctionGain)
    : correctionGain_(std::max(0.0f, correctionGain)) {}

Estimate BladeOrientationEstimator::push(const Sample& sample) {
  const float accelerationNorm = norm(sample.accelerationG);
  const bool gravityUsable = accelerationNorm >= kMinimumAccelNormG &&
                             accelerationNorm <= kMaximumAccelNormG;

  if (!initialized_) {
    if (!gravityUsable) return estimate();
    sensorOrientation_ = fromGravity(sample.accelerationG);
    lastTimestampUs_ = sample.nativeTimestampUs;
    acceptedSamples_ = 1;
    initialized_ = true;
    quality_ = Quality::Degraded;
    return estimate();
  }

  if (sample.nativeTimestampUs <= lastTimestampUs_) {
    quality_ = Quality::Unavailable;
    return estimate();
  }
  const uint64_t gapUs = sample.nativeTimestampUs - lastTimestampUs_;
  lastTimestampUs_ = sample.nativeTimestampUs;
  if (gapUs > kMaximumIntegrationGapUs) {
    quality_ = Quality::Stale;
    return estimate();
  }

  Vector3 correctedRate = sample.rotationRateRadiansPerSecond;
  if (gravityUsable) {
    const Vector3 error = cross(normalized(sample.accelerationG),
                                estimatedGravity(sensorOrientation_));
    correctedRate.x += correctionGain_ * error.x;
    correctedRate.y += correctionGain_ * error.y;
    correctedRate.z += correctionGain_ * error.z;
  }
  sensorOrientation_ = integrate(sensorOrientation_, correctedRate,
                                 static_cast<float>(gapUs) / 1000000.0f);
  ++acceptedSamples_;
  quality_ = !gravityUsable || acceptedSamples_ < kQualificationSamples
      ? Quality::Degraded
      : Quality::Qualified;
  return estimate();
}

bool BladeOrientationEstimator::setMountingReference() {
  if (!initialized_ || quality_ == Quality::Unavailable || quality_ == Quality::Stale) {
    return false;
  }
  mountingReferenceInverse_ = inverse(sensorOrientation_);
  mountingCalibrated_ = true;
  return true;
}

void BladeOrientationEstimator::reset() {
  sensorOrientation_ = {};
  mountingReferenceInverse_ = {};
  lastTimestampUs_ = 0;
  acceptedSamples_ = 0;
  quality_ = Quality::Unavailable;
  initialized_ = false;
  mountingCalibrated_ = false;
}

Estimate BladeOrientationEstimator::estimate() const {
  Estimate result{};
  result.equipmentOrientationQuaternion = mountingCalibrated_
      ? normalized(multiply(mountingReferenceInverse_, sensorOrientation_))
      : sensorOrientation_;
  result.orientationQuality = quality_;
  result.nativeTimestampUs = lastTimestampUs_;
  result.mountingCalibrated = mountingCalibrated_;
  return result;
}

Alignment relativeAlignment(const Estimate& left, const Estimate& right,
                            uint64_t maximumTimeSeparationUs) {
  Alignment result{};
  result.timeSeparationUs = left.nativeTimestampUs > right.nativeTimestampUs
      ? left.nativeTimestampUs - right.nativeTimestampUs
      : right.nativeTimestampUs - left.nativeTimestampUs;
  result.orientationQuality = lowerQuality(left.orientationQuality, right.orientationQuality);
  if (!left.mountingCalibrated || !right.mountingCalibrated ||
      result.orientationQuality == Quality::Unavailable ||
      result.orientationQuality == Quality::Stale ||
      result.timeSeparationUs > maximumTimeSeparationUs) {
    return result;
  }
  const Quaternion a = normalized(left.equipmentOrientationQuaternion);
  const Quaternion b = normalized(right.equipmentOrientationQuaternion);
  const float dot = std::abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
  result.relativeEquipmentAlignmentDegrees =
      2.0f * std::acos(std::clamp(dot, 0.0f, 1.0f)) * 180.0f / kPi;
  result.available = true;
  return result;
}

}  // namespace remus::orientation
