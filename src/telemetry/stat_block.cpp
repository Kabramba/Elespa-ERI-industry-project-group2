#include "telemetry/stat_block.h"

#include <math.h>

namespace telemetry {

StatBlock emptyStats() {
  StatBlock s{};
  s.min = s.max = s.mean = s.last = s.slopePerSec = NAN;
  s.stddev = 0.0f;
  s.samples = 0;
  s.windowMs = 0;
  s.reserved = 0;
  return s;
}

void RunningStat::reset(uint32_t nowMs) {
  samples_ = 0;
  mean_ = m2_ = min_ = max_ = first_ = last_ = 0.0f;
  startMs_ = lastMs_ = nowMs;
}

void RunningStat::push(float value, uint32_t nowMs) {
  if (samples_ == 0) {
    startMs_ = nowMs;
    first_ = min_ = max_ = value;
  } else {
    if (value < min_) min_ = value;
    if (value > max_) max_ = value;
  }

  // Welford: mean and M2 updated together so no large intermediate sum of
  // squares is ever formed.
  ++samples_;
  const float delta = value - mean_;
  mean_ += delta / static_cast<float>(samples_);
  m2_ += delta * (value - mean_);

  last_   = value;
  lastMs_ = nowMs;
}

StatBlock RunningStat::snapshot() const {
  if (samples_ == 0) return emptyStats();

  StatBlock s{};
  s.min      = min_;
  s.max      = max_;
  s.mean     = mean_;
  s.last     = last_;
  s.samples  = samples_;
  s.windowMs = lastMs_ - startMs_;
  s.reserved = 0;

  // Sample standard deviation. Undefined for a single sample, which is reported
  // as zero spread rather than NaN - one reading genuinely has no spread.
  s.stddev = (samples_ > 1) ? sqrtf(m2_ / static_cast<float>(samples_ - 1)) : 0.0f;

  // Trend across the window. Needs two samples and a non-zero span; a window
  // that collapsed to an instant would otherwise divide by zero.
  s.slopePerSec = (samples_ > 1 && s.windowMs > 0)
                      ? (last_ - first_) / (static_cast<float>(s.windowMs) / 1000.0f)
                      : 0.0f;
  return s;
}

}  // namespace telemetry
