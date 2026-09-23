#pragma once

#include <cmath>

namespace remus::metrics {

constexpr double kSplitDistanceMeters = 500.0;
constexpr double kMinMovingSpeedMps = 0.8;

inline bool splitValid(double speedMps) {
  return std::isfinite(speedMps) && speedMps >= kMinMovingSpeedMps;
}

inline double splitSeconds500m(double speedMps) {
  return splitValid(speedMps) ? (kSplitDistanceMeters / speedMps) : 0.0;
}

}  // namespace remus::metrics
