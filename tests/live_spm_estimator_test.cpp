#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "remus/core/LiveSpmEstimator.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHz = 25.0;
constexpr double kDt = 1.0 / kHz;

struct SignalGenerator {
  double current_time = 0.0;

  std::vector<remus::live::Result> feedCadence(
      remus::live::LiveSpmEstimator& estimator,
      double duration_seconds,
      double spm,
      double amplitude = 2.5,
      int axis = 0) {
    std::vector<remus::live::Result> updates;
    const double f = spm / 60.0;
    const int count = static_cast<int>(std::round(duration_seconds * kHz));
    for (int i = 0; i < count; ++i) {
      current_time += kDt;
      const double wave = amplitude * std::sin(2.0 * kPi * f * current_time);
      double x = 0.0, y = 0.0, z = 9.80665;
      if (axis == 0) {
        x = wave;
        y = 0.05 * wave;
      } else if (axis == 1) {
        y = wave;
        x = 0.05 * wave;
      } else {
        z += wave;
      }
      auto res = estimator.push(current_time, x, y, z);
      if (res.updated) {
        updates.push_back(res);
      }
    }
    return updates;
  }

  std::vector<remus::live::Result> feedQuiet(
      remus::live::LiveSpmEstimator& estimator,
      double duration_seconds) {
    std::vector<remus::live::Result> updates;
    const int count = static_cast<int>(std::round(duration_seconds * kHz));
    for (int i = 0; i < count; ++i) {
      current_time += kDt;
      auto res = estimator.push(current_time, 0.0, 0.0, 9.80665);
      if (res.updated) {
        updates.push_back(res);
      }
    }
    return updates;
  }

  std::vector<remus::live::Result> feedNoise(
      remus::live::LiveSpmEstimator& estimator,
      double duration_seconds,
      double amplitude = 0.8) {
    std::vector<remus::live::Result> updates;
    const int count = static_cast<int>(std::round(duration_seconds * kHz));
    unsigned int seed = 12345;
    auto lcg = [&]() {
      seed = seed * 1664525u + 1013904223u;
      return (static_cast<double>(seed % 10000) / 5000.0) - 1.0;
    };
    for (int i = 0; i < count; ++i) {
      current_time += kDt;
      const double noise_x = amplitude * lcg();
      const double noise_y = amplitude * lcg();
      auto res = estimator.push(current_time, noise_x, noise_y, 9.80665);
      if (res.updated) {
        updates.push_back(res);
      }
    }
    return updates;
  }

  std::vector<remus::live::Result> feedCompetingAxes(
      remus::live::LiveSpmEstimator& estimator,
      double duration_seconds,
      double spm_x,
      double spm_y) {
    std::vector<remus::live::Result> updates;
    const double fx = spm_x / 60.0;
    const double fy = spm_y / 60.0;
    const int count = static_cast<int>(std::round(duration_seconds * kHz));
    for (int i = 0; i < count; ++i) {
      current_time += kDt;
      const double x = 1.5 * std::sin(2.0 * kPi * fx * current_time);
      const double y = 1.5 * std::sin(2.0 * kPi * fy * current_time);
      auto res = estimator.push(current_time, x, y, 9.80665);
      if (res.updated) {
        updates.push_back(res);
      }
    }
    return updates;
  }
};

void testStableCadence18And24() {
  // Test 1: Stable 18 SPM
  {
    SignalGenerator gen;
    remus::live::LiveSpmEstimator estimator;
    auto updates = gen.feedCadence(estimator, 15.0, 18.0);
    assert(!updates.empty());
    const auto& last = updates.back();
    assert(last.available);
    assert(!last.held);
    assert(std::abs(last.stroke_rate_spm - 18.0) <= 1.0);
    assert(last.supported_timestamp > 0.0);
    assert(!remus::live::shouldClearLivePresentation(last));
  }

  // Test 2: Stable 24 SPM
  {
    SignalGenerator gen;
    remus::live::LiveSpmEstimator estimator;
    auto updates = gen.feedCadence(estimator, 15.0, 24.0);
    assert(!updates.empty());
    const auto& last = updates.back();
    assert(last.available);
    assert(!last.held);
    assert(std::abs(last.stroke_rate_spm - 24.0) <= 1.0);
    assert(last.supported_timestamp > 0.0);
    assert(!remus::live::shouldClearLivePresentation(last));
  }
  std::cout << "[PASS] testStableCadence18And24\n";
}

void testStateSequenceHoldAndQuietRelease() {
  SignalGenerator gen;
  remus::live::LiveSpmEstimator estimator;

  // 1. Establish stable 30 SPM for 15s
  auto initial_updates = gen.feedCadence(estimator, 15.0, 30.0, 3.0);
  assert(!initial_updates.empty());
  double last_available_supported_time = -1.0;
  for (const auto& u : initial_updates) {
    if (u.available && !u.held) {
      last_available_supported_time = u.supported_timestamp;
    }
  }
  assert(last_available_supported_time > 0.0);

  // 2. Feed quietude and trace state transitions: available -> held -> confirmed stop (recent_quiet)
  auto quiet_updates = gen.feedQuiet(estimator, 12.0);
  assert(!quiet_updates.empty());

  bool observed_held = false;
  bool observed_confirmed_stop = false;

  for (const auto& u : quiet_updates) {
    if (u.available && !u.held) {
      // Still available during early quietude while window is full of movement
      last_available_supported_time = u.supported_timestamp;
      assert(last_available_supported_time > 0.0);
    } else if (u.held) {
      // Enters held: holds last trusted cadence and keeps its supported timestamp frozen
      observed_held = true;
      assert(u.available);
      assert(std::abs(u.stroke_rate_spm - 30.0) <= 1.0);
      assert(u.supported_timestamp == last_available_supported_time);
      assert(!remus::live::shouldClearLivePresentation(u));
    } else if (!u.available) {
      if (u.reason == "recent_quiet" || u.reason == "insufficient_signal" || u.reason == "recent_weak_periodicity") {
        observed_confirmed_stop = true;
        assert(u.supported_timestamp < 0.0);
        assert(remus::live::shouldClearLivePresentation(u));
      }
    }
  }

  assert(observed_held);
  assert(observed_confirmed_stop);

  // 3. Reacquisition: resume 30 SPM -> returns to AVAILABLE
  auto reacquisition_updates = gen.feedCadence(estimator, 15.0, 30.0, 3.0);
  assert(!reacquisition_updates.empty());
  const auto& reacquired = reacquisition_updates.back();
  assert(reacquired.available);
  assert(!reacquired.held);
  assert(std::abs(reacquired.stroke_rate_spm - 30.0) <= 1.0);
  assert(reacquired.supported_timestamp > last_available_supported_time);
  assert(!remus::live::shouldClearLivePresentation(reacquired));

  std::cout << "[PASS] testStateSequenceHoldAndQuietRelease\n";
}

void testWeakPeriodicityAndHoldExpiry() {
  SignalGenerator gen;
  remus::live::LiveSpmEstimator estimator;

  // Establish 20 SPM
  gen.feedCadence(estimator, 15.0, 20.0);

  // Feed non-quiet noise (weak periodicity)
  auto noise_updates = gen.feedNoise(estimator, 8.0);
  assert(!noise_updates.empty());

  // Sequence: held -> unavailable
  bool observed_held = false;
  bool observed_unavailable = false;
  for (const auto& u : noise_updates) {
    if (u.held) observed_held = true;
    if (!u.available && !u.held) {
      observed_unavailable = true;
      assert(remus::live::shouldClearLivePresentation(u));
    }
  }
  assert(observed_held);
  assert(observed_unavailable);

  std::cout << "[PASS] testWeakPeriodicityAndHoldExpiry\n";
}

void testSampleGap() {
  SignalGenerator gen;
  remus::live::LiveSpmEstimator estimator;

  // Establish cadence
  gen.feedCadence(estimator, 15.0, 22.0);

  // Introduce a time jump (gap > 0.15s)
  gen.current_time += 1.0; // 1s gap
  estimator.push(gen.current_time, 1.0, 0.0, 9.80665);
  // Continue feeding for an update cycle
  auto post_gap = gen.feedCadence(estimator, 2.0, 22.0);
  assert(!post_gap.empty());
  bool saw_gap_or_held = false;
  for (const auto& u : post_gap) {
    if (u.reason.find("gap") != std::string::npos || u.held) {
      saw_gap_or_held = true;
      break;
    }
  }
  assert(saw_gap_or_held);
  std::cout << "[PASS] testSampleGap\n";
}

void testCompetingAxes() {
  SignalGenerator gen;
  remus::live::LiveSpmEstimator estimator;

  // Feed competing cadences on X (18 SPM) and Y (28 SPM) with equal amplitude
  auto updates = gen.feedCompetingAxes(estimator, 16.0, 18.0, 28.0);
  assert(!updates.empty());
  bool saw_competing_or_rejected = false;
  for (const auto& u : updates) {
    if (u.reason.find("competing") != std::string::npos || !u.available || u.held) {
      saw_competing_or_rejected = true;
      break;
    }
  }
  assert(saw_competing_or_rejected);
  std::cout << "[PASS] testCompetingAxes\n";
}

}  // namespace

int main() {
  testStableCadence18And24();
  testStateSequenceHoldAndQuietRelease();
  testWeakPeriodicityAndHoldExpiry();
  testSampleGap();
  testCompetingAxes();
  std::cout << "All live SPM estimator tests passed successfully!\n";
  return 0;
}
