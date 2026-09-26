#pragma once

#include <cstddef>
#include <cstdint>

#include "remus/core/BladeOrientation.hpp"

namespace remus::orientation {

enum class BladeSide : uint8_t { Left = 0, Right = 1 };

struct ClockMapping {
  double scale = 1.0;
  int64_t offsetMicroseconds = 0;
  uint64_t maximumErrorMicroseconds = 0;
  bool qualified = false;
};

struct BladeChannelSnapshot {
  uint32_t sourceIdentityHash = 0;
  BladeSide side = BladeSide::Left;
  Estimate orientation{};
  uint64_t alignedTimestampUs = 0;
  bool configured = false;
  bool clockAligned = false;
};

struct DualBladeSnapshot {
  BladeChannelSnapshot left{};
  BladeChannelSnapshot right{};
  Alignment alignment{};
};

class DualBladeOrientation {
 public:
  bool configure(BladeSide side, uint32_t sourceIdentityHash,
                 const ClockMapping& clockMapping);
  bool updateClockMapping(BladeSide side, const ClockMapping& clockMapping);
  Estimate push(BladeSide side, uint32_t sourceIdentityHash, const Sample& sample);
  bool calibrate(BladeSide side);
  void reset(BladeSide side);
  DualBladeSnapshot snapshot(uint64_t maximumPairSeparationUs = 20000) const;

 private:
  struct Channel {
    BladeOrientationEstimator estimator{};
    ClockMapping clockMapping{};
    uint32_t sourceIdentityHash = 0;
    BladeSide side = BladeSide::Left;
    bool configured = false;
  };

  Channel channels_[2]{};

  static size_t index(BladeSide side);
  static uint64_t alignedTimestamp(const Estimate& estimate,
                                   const ClockMapping& mapping);
  BladeChannelSnapshot channelSnapshot(const Channel& channel) const;
};

}  // namespace remus::orientation
