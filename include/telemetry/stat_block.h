// stat_block.h - turning a fast signal into something worth transmitting.
//
// Engine RPM moves faster than any sane telemetry link should carry it sample by
// sample, and the consumers downstream want features, not a waveform. So a fast
// channel is accumulated on the device and a STATISTICAL SUMMARY is sent once per
// window instead.
//
// RunningStat uses Welford's online algorithm: one pass, constant memory, and
// numerically stable where the naive sum-of-squares loses precision badly once
// the mean is large relative to the variance - which is exactly the case for RPM
// idling around 1500 with a spread of tens.
#pragma once

#include <stdint.h>

namespace telemetry {

// What a consumer receives for one fast channel over one window.
struct StatBlock {
  float min;          // NaN when samples == 0
  float max;
  float mean;
  float stddev;       // sample standard deviation; 0 when samples < 2
  float last;         // most recent raw sample, for a live readout
  float slopePerSec;  // (last - first) / window, a crude trend term
  uint32_t windowMs;  // time actually spanned, not the nominal period
  uint16_t samples;
  uint16_t reserved;
};

// Accumulator. Producer side only; never transmitted.
class RunningStat {
 public:
  void reset(uint32_t nowMs);
  void push(float value, uint32_t nowMs);

  bool     empty() const { return samples_ == 0; }
  uint16_t samples() const { return samples_; }

  // Summarise the window so far. Does not reset - call reset() to start the next.
  StatBlock snapshot() const;

 private:
  uint16_t samples_ = 0;
  float    mean_    = 0.0f;
  float    m2_      = 0.0f;  // sum of squared deviations from the running mean
  float    min_     = 0.0f;
  float    max_     = 0.0f;
  float    first_   = 0.0f;
  float    last_    = 0.0f;
  uint32_t startMs_ = 0;
  uint32_t lastMs_  = 0;
};

// A StatBlock with samples == 0, so an absent channel is unmistakable rather
// than reading as a plausible zero.
StatBlock emptyStats();

}  // namespace telemetry
