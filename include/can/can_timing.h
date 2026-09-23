// can_timing.h - CAN bit timing arithmetic.
//
// Pure: no Arduino, no ESP-IDF, no hardware. A CAN bit is divided into time
// quanta, and the controller samples it at a configurable point:
//
//     | SYNC (1 tq) |      TSEG1      |   TSEG2   |
//                                     ^ sample point
//
//     bit time (tq)  = 1 + tseg1 + tseg2
//     bitrate        = clock / (brp * bit time)
//     sample point % = (1 + tseg1) / bit time
//
// A later sample point tolerates more propagation delay, which is what matters
// on a long or slow-edged bus. Deriving this instead of hard-coding
// TWAI_TIMING_CONFIG_500KBITS() is what lets a profile name any bitrate.
#pragma once

#include <stdint.h>

namespace can {

// ESP32 / ESP32-S3 TWAI register limits.
constexpr uint32_t kTwaiClockHz = 80000000;  // APB
constexpr uint32_t kBrpMin      = 2;
constexpr uint32_t kBrpMax      = 16384;
constexpr uint8_t  kTseg1Min    = 1;
constexpr uint8_t  kTseg1Max    = 16;
constexpr uint8_t  kTseg2Min    = 1;
constexpr uint8_t  kTseg2Max    = 8;
constexpr uint8_t  kSjwMax      = 4;

struct BitTiming {
  uint32_t brp;
  uint8_t  tseg1;
  uint8_t  tseg2;
  uint8_t  sjw;
  bool     tripleSampling;

  // What the register values actually produce, which may differ from what was
  // asked for. Callers should check rather than assume.
  uint32_t timeQuanta;
  uint32_t actualBitrate;
  uint8_t  actualSamplePointPercent;
};

// Solve for register values giving exactly `bitrate` with a sample point as
// close to `wantSamplePercent` as the hardware allows. Only exact bitrates are
// accepted - an approximate one desynchronises a bus rather than degrading
// gracefully. Returns false if no legal combination exists.
bool computeBitTiming(uint32_t clockHz, uint32_t bitrate, uint8_t wantSamplePercent,
                      BitTiming &out);

// Convenience for the on-chip controller at its usual 80 MHz.
inline bool computeBitTiming(uint32_t bitrate, uint8_t wantSamplePercent, BitTiming &out) {
  return computeBitTiming(kTwaiClockHz, bitrate, wantSamplePercent, out);
}

}  // namespace can
