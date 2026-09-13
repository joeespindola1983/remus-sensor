#include "LiveSpmEstimator.hpp"

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

double correlation(const std::vector<Vec3>& x, std::size_t lag) {
  double dot = 0, a = 0, b = 0;
  for (std::size_t i = 0; i + lag < x.size(); ++i) for (int d = 0; d < 3; ++d) {
    dot += x[i][d] * x[i + lag][d];
    a += x[i][d] * x[i][d];
    b += x[i + lag][d] * x[i + lag][d];
  }
  const double denominator = std::sqrt(a * b);
  return denominator > 1e-30 ? std::clamp(dot / denominator, -1.0, 1.0) : 0;
}
}  // namespace

void LiveSpmEstimator::reset() {
  samples_.clear();
  first_timestamp_ = last_timestamp_ = last_estimate_timestamp_ = -1;
  filter_initialized_ = false;
  filter_stage_one_ = filter_stage_two_ = {};
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

Result LiveSpmEstimator::estimate(double now) const {
  Result result;
  result.updated = true;
  result.progress = 1;
  const double start = now - kWindowSeconds;
  auto begin = std::lower_bound(samples_.begin(), samples_.end(), start,
      [](const Sample& sample, double value) { return sample.timestamp < value; });
  if (begin == samples_.end() || samples_.back().timestamp - begin->timestamp < kWindowSeconds - .15) {
    result.reason = "sample_gap";
    return result;
  }
  const auto count = static_cast<double>(std::distance(begin, samples_.end()));
  result.observed_hz = count / (samples_.back().timestamp - begin->timestamp);
  if (result.observed_hz < 15 || result.observed_hz > 250) {
    result.reason = "unsupported_sample_rate";
    return result;
  }

  const std::size_t output_count = static_cast<std::size_t>(kWindowSeconds * kTargetHz);
  std::vector<Vec3> values;
  values.reserve(output_count);
  auto right = begin;
  for (std::size_t i = 0; i < output_count; ++i) {
    const double t = start + static_cast<double>(i) / kTargetHz;
    while (right != samples_.end() && right->timestamp < t) ++right;
    if (right == samples_.end()) { result.reason = "sample_gap"; return result; }
    if (std::abs(right->timestamp - t) < 1e-6) {
      values.push_back({right->x, right->y, right->z});
      continue;
    }
    if (right == samples_.begin()) { result.reason = "sample_gap"; return result; }
    const auto left = std::prev(right);
    const double gap = right->timestamp - left->timestamp;
    if (gap <= 0 || gap > .15) { result.reason = "sample_gap"; return result; }
    const double f = (t - left->timestamp) / gap;
    values.push_back({left->x + f * (right->x - left->x), left->y + f * (right->y - left->y), left->z + f * (right->z - left->z)});
  }
  values = detrend(std::move(values));
  double energy = 0;
  for (const auto& v : values) for (double a : v) energy += a * a;
  if (std::sqrt(energy / values.size()) < .02) { result.reason = "insufficient_signal"; return result; }

  const std::size_t min_lag = static_cast<std::size_t>(std::ceil(60 * kTargetHz / kMaxSpm));
  const std::size_t max_lag = static_cast<std::size_t>(std::floor(60 * kTargetHz / kMinSpm));
  struct Candidate { double lag, spm, score; bool boundary, redundant = false; };
  std::vector<double> acf(max_lag + 1);
  for (std::size_t lag = min_lag; lag <= max_lag; ++lag) acf[lag] = correlation(values, lag);
  std::vector<Candidate> candidates;
  for (std::size_t lag = min_lag; lag <= max_lag; ++lag) {
    if ((lag == min_lag || acf[lag] > acf[lag - 1]) && (lag == max_lag || acf[lag] > acf[lag + 1])) {
      double refined = static_cast<double>(lag);
      const bool boundary = lag == min_lag || lag == max_lag;
      if (!boundary) {
        const double denominator = acf[lag - 1] - 2 * acf[lag] + acf[lag + 1];
        if (std::abs(denominator) > 1e-12)
          refined += std::clamp(.5 * (acf[lag - 1] - acf[lag + 1]) / denominator, -.5, .5);
      }
      candidates.push_back({refined, 60 * kTargetHz / refined, acf[lag], boundary});
    }
  }
  for (auto& longer : candidates) for (const auto& shorter : candidates) {
    const double ratio = longer.lag / shorter.lag;
    if (shorter.score >= kMinCorrelation && ratio >= 1.96 && std::abs(ratio - std::round(ratio)) <= .04 &&
        shorter.score >= longer.score - .05) longer.redundant = true;
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
  std::vector<Candidate> eligible;
  for (const auto& candidate : candidates) if (!candidate.redundant && candidate.score >= kMinCorrelation) eligible.push_back(candidate);
  if (eligible.empty()) { result.reason = "weak_periodicity"; return result; }
  if (eligible.front().boundary) { result.reason = "search_boundary"; return result; }
  if (eligible.size() > 1 && eligible[0].score - eligible[1].score < .08 && std::abs(eligible[0].spm - eligible[1].spm) > 1) {
    result.reason = "competing_periods";
    return result;
  }
  result.available = true;
  result.stroke_rate_spm = eligible.front().spm;
  result.periodicity = eligible.front().score;
  result.reason.clear();
  return result;
}
}  // namespace remus::live
