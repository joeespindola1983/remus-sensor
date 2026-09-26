#include <cassert>
#include <cmath>
#include <iostream>

#include "remus/core/DualBladeOrientation.hpp"

using namespace remus::orientation;

int main() {
  DualBladeOrientation pair;
  const ClockMapping leftClock{1.0, 100000, 1000, true};
  const ClockMapping rightClock{1.0, -50000, 1000, true};
  assert(pair.configure(BladeSide::Left, 0x1111, leftClock));
  assert(pair.configure(BladeSide::Right, 0x2222, rightClock));
  assert(!pair.configure(BladeSide::Right, 0x1111, rightClock));

  for (uint64_t index = 0; index < 30; ++index) {
    pair.push(BladeSide::Left, 0x1111,
              {index * 5000, {0, 0, 1}, {0, 0, 0}});
    pair.push(BladeSide::Right, 0x2222,
              {150000 + index * 5000, {0, 0, 1}, {0, 0, 0}});
  }
  assert(pair.calibrate(BladeSide::Left));
  assert(pair.calibrate(BladeSide::Right));
  auto snapshot = pair.snapshot();
  assert(snapshot.alignment.available);
  assert(snapshot.alignment.timeSeparationUs == 0);

  constexpr float halfTurnRate = 3.14159265358979323846f / 2.0f;
  for (uint64_t index = 30; index < 230; ++index) {
    pair.push(BladeSide::Left, 0x1111,
              {index * 5000, {0, 0, 0}, {0, 0, halfTurnRate}});
    pair.push(BladeSide::Right, 0x2222,
              {150000 + index * 5000, {0, 0, 1}, {0, 0, 0}});
  }
  snapshot = pair.snapshot();
  assert(snapshot.alignment.available);
  assert(std::abs(snapshot.alignment.relativeEquipmentAlignmentDegrees - 90.0f) < 1.0f);

  assert(pair.updateClockMapping(BladeSide::Right, {1.0, 0, 25000, false}));
  assert(!pair.snapshot().alignment.available);
  assert(pair.push(BladeSide::Right, 0x9999, {}) .orientationQuality == Quality::Unavailable);

  std::cout << "dual_blade_orientation_smoke OK\n";
}
