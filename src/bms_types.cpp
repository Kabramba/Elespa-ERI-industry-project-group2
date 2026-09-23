#include "bms_types.h"

#include <math.h>

namespace jbd {

const char *sourceName(Source s) {
  switch (s) {
    case Source::kUart: return "UART";
    case Source::kCan:  return "CAN";
    case Source::kNone: return "none";
  }
  return "?";
}

const char *protectionName(uint8_t index) {
  static const char *const kNames[13] = {
      "CELL_OV", "CELL_UV", "PACK_OV", "PACK_UV", "CHG_OT", "CHG_UT", "DSG_OT",
      "DSG_UT",  "CHG_OC",  "DSG_OC",  "SHORT",   "AFE_ERR", "MOS_SW_LOCK"};
  return (index < 13) ? kNames[index] : nullptr;
}

void computeDerived(BmsSnapshot &s) {
  uint16_t lo = 0xFFFF, hi = 0;
  uint32_t sum = 0;
  uint8_t  live = 0, read = 0;

  const uint8_t n = (s.cellCount < kMaxCells) ? s.cellCount : kMaxCells;
  for (uint8_t i = 0; i < n; ++i) {
    // Never read this poll - a dropped frame, not a measurement.
    if ((s.cellsReadMask & (1u << i)) == 0) continue;
    ++read;
    const uint16_t mv = s.cellMilliVolts[i];
    sum += mv;
    // Read, but exactly 0 mV: a configured-but-unpopulated tap. Real, and
    // excluded from min/max so it cannot drag the spread to nonsense.
    if (mv == 0) continue;
    ++live;
    if (mv < lo) lo = mv;
    if (mv > hi) hi = mv;
  }

  s.cellsRead     = read;
  s.liveCellCount = live;
  s.cellMinMv     = live ? lo : 0;
  s.cellMaxMv     = live ? hi : 0;
  s.cellSpreadMv  = live ? static_cast<uint16_t>(hi - lo) : 0;
  s.cellSumVolts  = sum / 1000.0f;
}

bool snapshotsAgree(const BmsSnapshot &a, const BmsSnapshot &b,
                    float voltTolerance, uint16_t cellToleranceMv) {
  if (!a.valid || !b.valid) return false;
  if (a.cellCount != b.cellCount) return false;
  if (a.ntcCount != b.ntcCount) return false;
  if (fabsf(a.packVolts - b.packVolts) > voltTolerance) return false;

  const uint8_t n = (a.cellCount < kMaxCells) ? a.cellCount : kMaxCells;
  for (uint8_t i = 0; i < n; ++i) {
    // Skip channels one side has not read this cycle - a dropped CAN frame
    // leaves zeros behind and is not a disagreement.
    if (a.cellMilliVolts[i] == 0 || b.cellMilliVolts[i] == 0) continue;
    const int diff = static_cast<int>(a.cellMilliVolts[i]) - static_cast<int>(b.cellMilliVolts[i]);
    if (diff > static_cast<int>(cellToleranceMv) || diff < -static_cast<int>(cellToleranceMv))
      return false;
  }
  return true;
}

}  // namespace jbd
