#pragma once

#include <cstddef>
#include <array>
#include <deque>
#include <string>

namespace remus::live {

struct Sample { double timestamp, x, y, z; };
struct Result {
  bool updated = false;
  bool available = false;
  double stroke_rate_spm = 0;
  double periodicity = 0;
  double observed_hz = 0;
  double progress = 0;
  std::string reason = "collecting_window";
};

// Causal, source-local preview. This deliberately does not replace the
// evidence-oriented offline engine result stored after capture.
class LiveSpmEstimator {
 public:
  Result push(double timestamp, double x, double y, double z);
  void reset();

 private:
  Result estimate(double now) const;
  std::deque<Sample> samples_;
  double first_timestamp_ = -1;
  double last_timestamp_ = -1;
  double last_estimate_timestamp_ = -1;
  bool filter_initialized_ = false;
  std::array<double, 3> filter_stage_one_{};
  std::array<double, 3> filter_stage_two_{};
};

inline constexpr double kWindowSeconds = 15.0;
inline constexpr double kUpdateSeconds = 1.0;
inline constexpr const char* kAlgorithmVersion = "live-vector-acf-0.1-experimental";

}  // namespace remus::live
