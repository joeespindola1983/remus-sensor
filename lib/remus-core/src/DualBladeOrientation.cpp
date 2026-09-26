#include "remus/core/DualBladeOrientation.hpp"

#include <cmath>
#include <limits>

namespace remus::orientation {

size_t DualBladeOrientation::index(BladeSide side) {
  return side == BladeSide::Left ? 0U : 1U;
}

bool DualBladeOrientation::configure(BladeSide side, uint32_t sourceIdentityHash,
                                     const ClockMapping& clockMapping) {
  if (sourceIdentityHash == 0) return false;
  Channel& target = channels_[index(side)];
  const Channel& other = channels_[index(side == BladeSide::Left
      ? BladeSide::Right : BladeSide::Left)];
  if (other.configured && other.sourceIdentityHash == sourceIdentityHash) return false;
  if (target.configured && target.sourceIdentityHash != sourceIdentityHash) {
    target.estimator.reset();
  }
  target.sourceIdentityHash = sourceIdentityHash;
  target.side = side;
  target.clockMapping = clockMapping;
  target.configured = true;
  return true;
}

bool DualBladeOrientation::updateClockMapping(BladeSide side,
                                              const ClockMapping& clockMapping) {
  Channel& channel = channels_[index(side)];
  if (!channel.configured) return false;
  channel.clockMapping = clockMapping;
  return true;
}

Estimate DualBladeOrientation::push(BladeSide side, uint32_t sourceIdentityHash,
                                    const Sample& sample) {
  Channel& channel = channels_[index(side)];
  if (!channel.configured || channel.sourceIdentityHash != sourceIdentityHash) {
    return {};
  }
  return channel.estimator.push(sample);
}

bool DualBladeOrientation::calibrate(BladeSide side) {
  Channel& channel = channels_[index(side)];
  return channel.configured && channel.estimator.setMountingReference();
}

void DualBladeOrientation::reset(BladeSide side) {
  channels_[index(side)] = Channel{};
}

uint64_t DualBladeOrientation::alignedTimestamp(const Estimate& estimate,
                                                const ClockMapping& mapping) {
  if (!mapping.qualified) return 0;
  const double mapped = mapping.scale * static_cast<double>(estimate.nativeTimestampUs) +
                        static_cast<double>(mapping.offsetMicroseconds);
  if (!std::isfinite(mapped) || mapped < 0.0 ||
      mapped > static_cast<double>(std::numeric_limits<uint64_t>::max())) return 0;
  return static_cast<uint64_t>(std::llround(mapped));
}

BladeChannelSnapshot DualBladeOrientation::channelSnapshot(const Channel& channel) const {
  BladeChannelSnapshot result{};
  result.sourceIdentityHash = channel.sourceIdentityHash;
  result.side = channel.side;
  result.orientation = channel.estimator.estimate();
  result.alignedTimestampUs = alignedTimestamp(result.orientation, channel.clockMapping);
  result.configured = channel.configured;
  result.clockAligned = channel.clockMapping.qualified && result.alignedTimestampUs > 0;
  return result;
}

DualBladeSnapshot DualBladeOrientation::snapshot(uint64_t maximumPairSeparationUs) const {
  DualBladeSnapshot result{};
  result.left = channelSnapshot(channels_[0]);
  result.right = channelSnapshot(channels_[1]);
  if (!result.left.configured || !result.right.configured ||
      !result.left.clockAligned || !result.right.clockAligned) return result;

  Estimate left = result.left.orientation;
  Estimate right = result.right.orientation;
  left.nativeTimestampUs = result.left.alignedTimestampUs;
  right.nativeTimestampUs = result.right.alignedTimestampUs;
  const uint64_t uncertainty = channels_[0].clockMapping.maximumErrorMicroseconds +
                               channels_[1].clockMapping.maximumErrorMicroseconds;
  if (uncertainty >= maximumPairSeparationUs) return result;
  result.alignment = relativeAlignment(
      left, right, maximumPairSeparationUs - uncertainty);
  if (result.alignment.available && uncertainty > 5000 &&
      result.alignment.orientationQuality == Quality::Qualified) {
    result.alignment.orientationQuality = Quality::Degraded;
  }
  return result;
}

}  // namespace remus::orientation
