#include "can/can_timing.h"

namespace can {
namespace {

inline int absDiff(int a, int b) { return (a > b) ? (a - b) : (b - a); }

}  // namespace

bool computeBitTiming(uint32_t clockHz, uint32_t bitrate, uint8_t wantSamplePercent,
                      BitTiming &out) {
  if (clockHz == 0 || bitrate == 0) return false;
  if (wantSamplePercent < 50) wantSamplePercent = 50;
  if (wantSamplePercent > 95) wantSamplePercent = 95;

  bool found   = false;
  int  bestErr = 0;

  // Walk from the most time quanta down. More quanta give finer control over
  // the sample point and better resynchronisation, so the first exact hit at a
  // high tq is the one worth taking.
  const uint32_t kMaxTq = 1u + kTseg1Max + kTseg2Max;  // 25 on this hardware
  for (uint32_t tq = kMaxTq; tq >= 8; --tq) {
    const uint32_t denom = bitrate * tq;
    if (denom == 0) continue;
    if (clockHz % denom != 0) continue;  // exact bitrates only
    const uint32_t brp = clockHz / denom;
    if (brp < kBrpMin || brp > kBrpMax) continue;
    if (brp & 1u) continue;  // the prescaler is even

    // Place the sample point, then clamp into the register limits, rederiving
    // the partner segment so the bit still totals tq quanta.
    uint32_t afterSync = (tq * wantSamplePercent + 50u) / 100u;
    if (afterSync < 2) afterSync = 2;
    if (afterSync > tq - 1) afterSync = tq - 1;

    uint32_t tseg1 = afterSync - 1u;
    uint32_t tseg2 = tq - 1u - tseg1;
    if (tseg2 < kTseg2Min) {
      tseg2 = kTseg2Min;
      tseg1 = tq - 1u - tseg2;
    }
    if (tseg2 > kTseg2Max) {
      tseg2 = kTseg2Max;
      tseg1 = tq - 1u - tseg2;
    }
    if (tseg1 < kTseg1Min || tseg1 > kTseg1Max) continue;

    const uint32_t actualPct = (1u + tseg1) * 100u / tq;
    const int      err       = absDiff(static_cast<int>(actualPct), static_cast<int>(wantSamplePercent));
    if (!found || err < bestErr) {
      found                        = true;
      bestErr                      = err;
      out.brp                      = brp;
      out.tseg1                    = static_cast<uint8_t>(tseg1);
      out.tseg2                    = static_cast<uint8_t>(tseg2);
      // SJW may not exceed either segment, nor the hardware maximum.
      uint32_t sjw                 = (tseg1 < tseg2) ? tseg1 : tseg2;
      if (sjw > kSjwMax) sjw       = kSjwMax;
      out.sjw                      = static_cast<uint8_t>(sjw ? sjw : 1);
      out.tripleSampling           = false;
      out.timeQuanta               = tq;
      out.actualBitrate            = clockHz / (brp * tq);
      out.actualSamplePointPercent = static_cast<uint8_t>(actualPct);
    }
    if (found && bestErr == 0) break;
  }
  return found;
}

}  // namespace can
