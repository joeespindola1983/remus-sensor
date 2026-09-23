#include <cassert>
#include <cmath>
#include <iostream>

#include "remus/core/LiveSpmEstimator.hpp"
#include "remus/core/SessionFormat.hpp"
#include "remus/core/SplitCalculator.hpp"

int main() {
  static_assert(sizeof(remus::session::FileHeader) == 32);
  static_assert(sizeof(remus::session::ImuRecord) == 17);
  static_assert(sizeof(remus::session::GpsRecord) == 19);
  static_assert(sizeof(remus::session::GpsV2Record) == 41);
  static_assert(sizeof(remus::session::SpmRecord) == 7);

  const auto v2 = remus::session::makeHeaderV2(123, 456, 200);
  assert(std::memcmp(v2.magic, "RBP2", 4) == 0);
  assert(v2.version == 2);

  assert(!remus::metrics::splitValid(0.79));
  assert(remus::metrics::splitValid(2.0));
  assert(std::abs(remus::metrics::splitSeconds500m(2.0) - 250.0) < 1e-9);

  remus::live::LiveSpmEstimator estimator;
  remus::live::Result last;
  constexpr double hz = 25.0;
  constexpr double spm = 30.0;
  constexpr double f = spm / 60.0;
  for (int i = 0; i < 16 * static_cast<int>(hz); ++i) {
    const double t = i / hz;
    const double x = 1.5 * std::sin(2.0 * 3.141592653589793 * f * t);
    last = estimator.push(t, x, 0.1 * x, 9.80665);
  }
  assert(last.available || last.held || last.progress >= 1.0);
  std::cout << "core_smoke OK\n";
}
