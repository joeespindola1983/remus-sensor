#include "remus/core/LiveSpmEstimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace remus::live {
namespace {
using Vec3 = std::array<double, 3>;
constexpr double kTargetHz = 25.0;
constexpr double kMinSpm = 12.0;
constexpr double kMaxSpm = 60.0;
constexpr double kMinCorrelation = 0.55;
constexpr double kMinAxisRms = 0.03;          // m/s^2 after detrend
constexpr double kMinAxisEnergyFraction = 0.08;
constexpr double kAxisAgreementBonus = 0.06;
constexpr double kContinuityBonus = 0.06;
constexpr double kPreferredAxisBonus = 0.04;
constexpr double kCompetingScoreDelta = 0.05;
// A long window estimates cadence robustly, but it contains old strokes for many
// seconds after the athlete stops. A second, short window must still support the
// selected cadence before a NEW live SPM can be published. This is deliberately
// a confirmation gate, not a second independent cadence estimator.
constexpr double kRecentMinCorrelation = 0.35;
constexpr double kRecentMinAxisRms = 0.03;
constexpr double kRecentLagToleranceFraction = 0.12;

std::vector<Vec3> detrend(std::vector<Vec3> values) {
  const double center = (values.size() - 1) * .5;
  Vec3 mean{}, slope{};
  double ss = 0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double t = static_cast<double>(i) - center;
    ss += t * t;
    for (int d = 0; d < 3; ++d) {
      mean[d] += values[i][d] / values.size();
      slope[d] += t * values[i][d];
    }
  }
  for (std::size_t i = 0; i < values.size(); ++i)
    for (int d = 0; d < 3; ++d)
      values[i][d] -= mean[d] + (static_cast<double>(i) - center) * slope[d] / ss;
  return values;
}

double correlationAxis(const std::vector<Vec3>& x, std::size_t lag, int axis) {
  double dot = 0, a = 0, b = 0;
  for (std::size_t i = 0; i + lag < x.size(); ++i) {
    const double lhs = x[i][axis];
    const double rhs = x[i + lag][axis];
    dot += lhs * rhs;
    a += lhs * lhs;
    b += rhs * rhs;
  }
  const double denominator = std::sqrt(a * b);
  return denominator > 1e-30 ? std::clamp(dot / denominator, -1.0, 1.0) : 0;
}

double spmTolerance(double spm) {
  return std::max(2.0, std::abs(spm) * 0.08);
}

double continuityTolerance(double spm) {
  return std::max(3.0, std::abs(spm) * 0.15);
}

struct AxisCandidate {
  int axis = -1;
  double spm = 0;
  double score = 0;
  double adjusted_score = 0;
  double rms = 0;
};

struct RecentSupport {
  double rms = 0;
  double score = 0;
  double spm = 0;
  bool enough_motion = false;
  bool cadence_matches = false;
};

RecentSupport evaluateRecentSupport(
    const std::vector<Vec3>& full_values, int axis, double expected_spm) {
  RecentSupport support;
  if (axis < 0 || axis > 2 || expected_spm <= 0 || full_values.empty()) return support;

  const std::size_t recent_count = std::min(
      full_values.size(),
      static_cast<std::size_t>(std::round(kRecentWindowSeconds * kTargetHz)));
  if (recent_count < 16) return support;

  std::vector<Vec3> recent(
      full_values.end() - static_cast<std::ptrdiff_t>(recent_count),
      full_values.end());
  recent = detrend(std::move(recent));

  double energy = 0;
  for (const auto& v : recent) energy += v[axis] * v[axis];
  support.rms = std::sqrt(energy / recent.size());
  support.enough_motion = support.rms >= kRecentMinAxisRms;
  if (!support.enough_motion) return support;

  const double expected_lag = 60.0 * kTargetHz / expected_spm;
  std::size_t lag_lo = static_cast<std::size_t>(std::floor(
      expected_lag * (1.0 - kRecentLagToleranceFraction)));
  std::size_t lag_hi = static_cast<std::size_t>(std::ceil(
      expected_lag * (1.0 + kRecentLagToleranceFraction)));
  lag_lo = std::max<std::size_t>(1, lag_lo);
  if (recent.size() < 3 || lag_lo >= recent.size() - 1) return support;
  lag_hi = std::min<std::size_t>(lag_hi, recent.size() - 2);
  if (lag_hi < lag_lo) return support;

  double best_score = -1.0;
  std::size_t best_lag = lag_lo;
  for (std::size_t lag = lag_lo; lag <= lag_hi; ++lag) {
    const double score = correlationAxis(recent, lag, axis);
    if (score > best_score) {
      best_score = score;
      best_lag = lag;
    }
  }

  support.score = std::max(0.0, best_score);
  support.spm = 60.0 * kTargetHz / static_cast<double>(best_lag);
  support.cadence_matches =
      support.score >= kRecentMinCorrelation &&
      std::abs(support.spm - expected_spm) <= spmTolerance(expected_spm);
  return support;
}

}  // namespace

void LiveSpmEstimator::reset() {
  samples_.clear();
  first_timestamp_ = last_timestamp_ = last_estimate_timestamp_ = -1;
  filter_initialized_ = false;
  filter_stage_one_ = filter_stage_two_ = {};
  last_valid_spm_ = 0;
  last_valid_timestamp_ = -1;
  quiet_since_timestamp_ = -1;
}

void LiveSpmEstimator::setPreferredAxis(int axis) {
  preferred_axis_ = (axis >= 0 && axis <= 2) ? axis : -1;
}

Result LiveSpmEstimator::push(double timestamp, double x, double y, double z) {
  Result result;
  if (!std::isfinite(timestamp) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      (last_timestamp_ >= 0 && timestamp <= last_timestamp_)) {
    result.updated = true;
    result.reason = "invalid_sample";
    return result;
  }
  const double dt = last_timestamp_ < 0 ? 0 : timestamp - last_timestamp_;
  if (first_timestamp_ < 0) first_timestamp_ = timestamp;
  last_timestamp_ = timestamp;
  const std::array<double, 3> input{x, y, z};
  if (!filter_initialized_) {
    filter_stage_one_ = filter_stage_two_ = input;
    filter_initialized_ = true;
  } else {
    // Two causal 3 Hz low-pass stages suppress handling vibration and avoid
    // aliasing before the common 25 Hz analysis grid.
    const double alpha = 1.0 - std::exp(-2.0 * 3.141592653589793 * 3.0 * dt);
    for (int d = 0; d < 3; ++d) {
      filter_stage_one_[d] += alpha * (input[d] - filter_stage_one_[d]);
      filter_stage_two_[d] += alpha * (filter_stage_one_[d] - filter_stage_two_[d]);
    }
  }
  samples_.push_back({timestamp, filter_stage_two_[0], filter_stage_two_[1], filter_stage_two_[2]});
  while (!samples_.empty() && samples_.front().timestamp < timestamp - kWindowSeconds - 1.0)
    samples_.pop_front();

  const double collected = timestamp - first_timestamp_;
  result.progress = std::clamp(collected / kWindowSeconds, 0.0, 1.0);
  if (collected < kWindowSeconds) {
    if (last_estimate_timestamp_ < 0 || timestamp - last_estimate_timestamp_ >= kUpdateSeconds) {
      last_estimate_timestamp_ = timestamp;
      result.updated = true;
    }
    return result;
  }
  if (last_estimate_timestamp_ >= 0 && timestamp - last_estimate_timestamp_ < kUpdateSeconds) return result;
  last_estimate_timestamp_ = timestamp;
  return estimate(timestamp);
}

Result LiveSpmEstimator::holdOrRelease(Result result, double now, bool quiet) {
  if (quiet) {
    if (quiet_since_timestamp_ < 0) quiet_since_timestamp_ = now;
  } else {
    quiet_since_timestamp_ = -1;
  }

  const bool recent_lock = last_valid_timestamp_ >= 0 &&
      (now - last_valid_timestamp_) <= kHoldLastValidSeconds;
  const bool quiet_long_enough = quiet_since_timestamp_ >= 0 &&
      (now - quiet_since_timestamp_) >= kQuietReleaseSeconds;

  if (recent_lock && !quiet_long_enough) {
    result.available = true;
    result.held = true;
    result.stroke_rate_spm = last_valid_spm_;
    result.selected_axis = -1;
    result.reason = "held_" + result.reason;
    return result;
  }

  if (!recent_lock || quiet_long_enough) {
    last_valid_spm_ = 0;
    last_valid_timestamp_ = -1;
  }
  return result;
}

Result LiveSpmEstimator::estimate(double now) {
  Result result;
  result.updated = true;
  result.progress = 1;
  const double start = now - kWindowSeconds;
  auto begin = std::lower_bound(samples_.begin(), samples_.end(), start,
      [](const Sample& sample, double value) { return sample.timestamp < value; });
  if (begin == samples_.end() || samples_.back().timestamp - begin->timestamp < kWindowSeconds - .15) {
    result.reason = "sample_gap";
    return holdOrRelease(std::move(result), now, false);
  }
  const auto count = static_cast<double>(std::distance(begin, samples_.end()));
  result.observed_hz = count / (samples_.back().timestamp - begin->timestamp);
  if (result.observed_hz < 15 || result.observed_hz > 250) {
    result.reason = "unsupported_sample_rate";
    return holdOrRelease(std::move(result), now, false);
  }

  const std::size_t output_count = static_cast<std::size_t>(kWindowSeconds * kTargetHz);
  std::vector<Vec3> values;
  values.reserve(output_count);
  auto right = begin;
  for (std::size_t i = 0; i < output_count; ++i) {
    const double t = start + static_cast<double>(i) / kTargetHz;
    while (right != samples_.end() && right->timestamp < t) ++right;
    if (right == samples_.end()) {
      result.reason = "sample_gap";
      return holdOrRelease(std::move(result), now, false);
    }
    if (std::abs(right->timestamp - t) < 1e-6) {
      values.push_back({right->x, right->y, right->z});
      continue;
    }
    if (right == samples_.begin()) {
      result.reason = "sample_gap";
      return holdOrRelease(std::move(result), now, false);
    }
    const auto left = std::prev(right);
    const double gap = right->timestamp - left->timestamp;
    if (gap <= 0 || gap > .15) {
      result.reason = "sample_gap";
      return holdOrRelease(std::move(result), now, false);
    }
    const double f = (t - left->timestamp) / gap;
    values.push_back({left->x + f * (right->x - left->x),
                      left->y + f * (right->y - left->y),
                      left->z + f * (right->z - left->z)});
  }

  values = detrend(std::move(values));

  std::array<double, 3> axis_rms{};
  double max_axis_rms = 0;
  for (int axis = 0; axis < 3; ++axis) {
    double energy = 0;
    for (const auto& v : values) energy += v[axis] * v[axis];
    axis_rms[axis] = std::sqrt(energy / values.size());
    max_axis_rms = std::max(max_axis_rms, axis_rms[axis]);
  }
  result.signal_rms = max_axis_rms;
  if (max_axis_rms < kMinAxisRms) {
    result.reason = "insufficient_signal";
    return holdOrRelease(std::move(result), now, true);
  }
  quiet_since_timestamp_ = -1;

  const std::size_t min_lag = static_cast<std::size_t>(std::ceil(60 * kTargetHz / kMaxSpm));
  const std::size_t max_lag = static_cast<std::size_t>(std::floor(60 * kTargetHz / kMinSpm));

  std::vector<AxisCandidate> axis_candidates;
  double best_raw_score = -1;
  double best_raw_spm = 0;
  int best_raw_axis = -1;

  for (int axis = 0; axis < 3; ++axis) {
    if (axis_rms[axis] < kMinAxisRms || axis_rms[axis] < max_axis_rms * kMinAxisEnergyFraction) continue;

    struct Candidate { double lag, spm, score; bool boundary, redundant = false; };
    std::vector<double> acf(max_lag + 1);
    for (std::size_t lag = min_lag; lag <= max_lag; ++lag) {
      acf[lag] = correlationAxis(values, lag, axis);
    }

    std::vector<Candidate> candidates;
    for (std::size_t lag = min_lag; lag <= max_lag; ++lag) {
      if ((lag == min_lag || acf[lag] > acf[lag - 1]) &&
          (lag == max_lag || acf[lag] > acf[lag + 1])) {
        double refined = static_cast<double>(lag);
        const bool boundary = lag == min_lag || lag == max_lag;
        if (!boundary) {
          const double denominator = acf[lag - 1] - 2 * acf[lag] + acf[lag + 1];
          if (std::abs(denominator) > 1e-12) {
            refined += std::clamp(.5 * (acf[lag - 1] - acf[lag + 1]) / denominator, -.5, .5);
          }
        }
        candidates.push_back({refined, 60 * kTargetHz / refined, acf[lag], boundary});
      }
    }

    for (auto& longer : candidates) for (const auto& shorter : candidates) {
      const double ratio = longer.lag / shorter.lag;
      if (shorter.score >= kMinCorrelation && ratio >= 1.96 &&
          std::abs(ratio - std::round(ratio)) <= .04 &&
          shorter.score >= longer.score - .05) {
        longer.redundant = true;
      }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

    if (!candidates.empty() && candidates.front().score > best_raw_score) {
      best_raw_score = candidates.front().score;
      best_raw_spm = candidates.front().spm;
      best_raw_axis = axis;
    }

    std::vector<Candidate> eligible;
    for (const auto& candidate : candidates) {
      if (!candidate.redundant && !candidate.boundary && candidate.score >= kMinCorrelation) {
        eligible.push_back(candidate);
      }
    }
    if (eligible.empty()) continue;

    // If one physical axis contains two almost-equally strong, clearly different
    // periods, that axis is ambiguous for this window and should not drive UI.
    if (eligible.size() > 1 && eligible[0].score - eligible[1].score < .08 &&
        std::abs(eligible[0].spm - eligible[1].spm) > 1.0) {
      continue;
    }

    AxisCandidate axis_candidate;
    axis_candidate.axis = axis;
    axis_candidate.spm = eligible.front().spm;
    axis_candidate.score = eligible.front().score;
    axis_candidate.adjusted_score = eligible.front().score;
    axis_candidate.rms = axis_rms[axis];
    axis_candidates.push_back(axis_candidate);
  }

  result.candidate_spm = best_raw_spm;
  result.periodicity = std::max(0.0, best_raw_score);
  result.selected_axis = best_raw_axis;

  if (axis_candidates.empty()) {
    result.reason = "weak_periodicity";
    return holdOrRelease(std::move(result), now, false);
  }

  // Reward agreement across independent axes. This keeps the estimator
  // orientation-agnostic without averaging noisy axes into a single ACF score.
  for (auto& candidate : axis_candidates) {
    for (const auto& other : axis_candidates) {
      if (candidate.axis == other.axis) continue;
      if (std::abs(candidate.spm - other.spm) <= spmTolerance(candidate.spm)) {
        candidate.adjusted_score += kAxisAgreementBonus;
      }
    }

    // A known mounting orientation can be supplied as a soft preference. It is
    // never a hard requirement: another axis wins if its periodicity is better.
    if (candidate.axis == preferred_axis_) candidate.adjusted_score += kPreferredAxisBonus;

    // Cadence continuity is also a soft tie-breaker only. It never promotes a
    // candidate that failed the 0.55 periodicity gate in the first place.
    if (last_valid_spm_ > 0 &&
        std::abs(candidate.spm - last_valid_spm_) <= continuityTolerance(last_valid_spm_)) {
      candidate.adjusted_score += kContinuityBonus;
    }
  }

  std::sort(axis_candidates.begin(), axis_candidates.end(),
      [](const AxisCandidate& a, const AxisCandidate& b) {
        return a.adjusted_score > b.adjusted_score;
      });

  if (axis_candidates.size() > 1) {
    const auto& first = axis_candidates[0];
    const auto& second = axis_candidates[1];
    const bool disagree = std::abs(first.spm - second.spm) > spmTolerance(first.spm);
    const bool too_close = first.adjusted_score - second.adjusted_score < kCompetingScoreDelta;
    if (disagree && too_close) {
      result.candidate_spm = first.spm;
      result.periodicity = first.score;
      result.selected_axis = first.axis;
      result.reason = "competing_axes";
      return holdOrRelease(std::move(result), now, false);
    }
  }

  const AxisCandidate& selected = axis_candidates.front();

  // Do not publish a new cadence only because it is strong somewhere inside
  // the 15-second history. The same cadence must still be present in the newest
  // few seconds. This prevents a stopped boat/device from producing rising SPM
  // as old strokes slide out of the long ACF window.
  const RecentSupport recent = evaluateRecentSupport(values, selected.axis, selected.spm);
  result.recent_signal_rms = recent.rms;
  result.recent_periodicity = recent.score;
  result.candidate_spm = selected.spm;
  result.periodicity = selected.score;
  result.selected_axis = selected.axis;

  if (!recent.enough_motion) {
    result.reason = "recent_quiet";
    return holdOrRelease(std::move(result), now, true);
  }
  if (!recent.cadence_matches) {
    result.reason = recent.score < kRecentMinCorrelation
        ? "recent_weak_periodicity"
        : "recent_cadence_mismatch";
    return holdOrRelease(std::move(result), now, false);
  }

  result.available = true;
  result.held = false;
  result.stroke_rate_spm = selected.spm;
  result.reason.clear();

  last_valid_spm_ = selected.spm;
  last_valid_timestamp_ = now;
  quiet_since_timestamp_ = -1;
  return result;
}

}  // namespace remus::live
