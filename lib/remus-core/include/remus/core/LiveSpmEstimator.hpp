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
  bool held = false;              // true = UI is holding the last trusted cadence
  double stroke_rate_spm = 0;
  double candidate_spm = 0;       // best raw candidate even when rejected
  double periodicity = 0;
  double signal_rms = 0;
  double recent_signal_rms = 0; // motion energy in the newest short window
  double recent_periodicity = 0; // support for the chosen cadence in that window
  double observed_hz = 0;
  double progress = 0;
  int selected_axis = -1;         // -1 unknown/held, 0=X, 1=Y, 2=Z
  std::string reason = "collecting_window";
};

// Causal, source-local preview. This deliberately does not replace the
// evidence-oriented offline engine result stored after capture.
//
// The estimator is mounting-agnostic by default: X/Y/Z are evaluated
// independently and the most trustworthy periodic axis wins. A known mounting
// can be supplied only as a soft preference; it never disables the other axes.
class LiveSpmEstimator {
 public:
  Result push(double timestamp, double x, double y, double z);
  void reset();
  void setPreferredAxis(int axis);  // -1=auto, 0=X, 1=Y, 2=Z

 private:
  Result estimate(double now);
  Result holdOrRelease(Result result, double now, bool quiet);

  std::deque<Sample> samples_;
  double first_timestamp_ = -1;
  double last_timestamp_ = -1;
  double last_estimate_timestamp_ = -1;
  bool filter_initialized_ = false;
  std::array<double, 3> filter_stage_one_{};
  std::array<double, 3> filter_stage_two_{};

  int preferred_axis_ = -1;
  double last_valid_spm_ = 0;
  double last_valid_timestamp_ = -1;
  double quiet_since_timestamp_ = -1;
};

inline constexpr double kWindowSeconds = 10.0;
inline constexpr double kUpdateSeconds = 1.5;
inline constexpr double kHoldLastValidSeconds = 2.5;
inline constexpr double kQuietReleaseSeconds = 2.5;
inline constexpr double kRecentWindowSeconds = 6.0;
inline constexpr const char* kAlgorithmVersion = "live-adaptive-axis-acf-0.5";

}  // namespace remus::live
