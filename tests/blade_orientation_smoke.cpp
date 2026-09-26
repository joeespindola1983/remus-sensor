#include <cassert>
#include <cmath>
#include <iostream>

#include "remus/core/BladeOrientation.hpp"

using remus::orientation::BladeOrientationEstimator;
using remus::orientation::Quality;
using remus::orientation::Sample;

int main() {
  BladeOrientationEstimator left(0.0f);
  BladeOrientationEstimator right(0.0f);

  for (uint64_t index = 0; index < 30; ++index) {
    const uint64_t timestamp = index * 5000;
    left.push(Sample{timestamp, {0, 0, 1}, {0, 0, 0}});
    right.push(Sample{timestamp, {0, 0, 1}, {0, 0, 0}});
  }
  assert(left.estimate().orientationQuality == Quality::Qualified);
  assert(left.setMountingReference());
  assert(right.setMountingReference());

  auto aligned = remus::orientation::relativeAlignment(left.estimate(), right.estimate());
  assert(aligned.available);
  assert(std::abs(aligned.relativeEquipmentAlignmentDegrees) < 0.01f);

  constexpr float radiansPerSecond = 3.14159265358979323846f / 2.0f;
  for (uint64_t index = 30; index < 230; ++index) {
    const uint64_t timestamp = index * 5000;
    left.push(Sample{timestamp, {0, 0, 0}, {0, 0, radiansPerSecond}});
    right.push(Sample{timestamp, {0, 0, 1}, {0, 0, 0}});
  }
  const auto separated = remus::orientation::relativeAlignment(
      left.estimate(), right.estimate());
  assert(separated.available);
  assert(separated.orientationQuality == Quality::Degraded);
  assert(std::abs(separated.relativeEquipmentAlignmentDegrees - 90.0f) < 1.0f);

  const auto unalignedTime = remus::orientation::relativeAlignment(
      left.estimate(),
      {right.estimate().equipmentOrientationQuaternion, Quality::Qualified,
       right.estimate().nativeTimestampUs + 50000, true});
  assert(!unalignedTime.available);

  std::cout << "blade_orientation_smoke OK\n";
}
