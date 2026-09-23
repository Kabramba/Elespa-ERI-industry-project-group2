// bms_types.h - the shared data model.
//
// Deliberately free of Arduino and ESP-IDF headers so that everything which
// touches only this file can be compiled and unit-tested on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace jbd {

constexpr uint8_t kMaxCells = 32;
constexpr uint8_t kMaxNtc   = 8;

// Protection bits, identical on the UART (register 0x03 offset 16) and the
// CAN (id 0x102) sides of the protocol.
enum Protection : uint16_t {
  kCellOv    = 1u << 0,
  kCellUv    = 1u << 1,
  kPackOv    = 1u << 2,
  kPackUv    = 1u << 3,
  kChgOt     = 1u << 4,
  kChgUt     = 1u << 5,
  kDsgOt     = 1u << 6,
  kDsgUt     = 1u << 7,
  kChgOc     = 1u << 8,
  kDsgOc     = 1u << 9,
  kShort     = 1u << 10,
  kAfeErr    = 1u << 11,
  kMosSwLock = 1u << 12,
};

enum class Source : uint8_t { kNone = 0, kUart = 1, kCan = 2 };

// Which groups of fields a poll actually obtained. On a lossy link some replies
// go missing, and a missing reply leaves its fields at zero - indistinguishable
// from a genuine zero unless we record what arrived. A consumer must check the
// relevant bit before believing a field.
enum Field : uint16_t {
  kFieldPack     = 1u << 0,  // packVolts, currentAmps, remainingAh
  kFieldCapacity = 1u << 1,  // fullAh, cycles, socPercent
  kFieldProtect  = 1u << 2,  // protection, balanceBits
  kFieldMos      = 1u << 3,  // MOS states, mfg date, softwareVersion
  kFieldConfig   = 1u << 4,  // cellCount, ntcCount
  kFieldTemps    = 1u << 5,  // ntcCelsius
  kFieldCells    = 1u << 6,  // at least one cell frame arrived
};

const char *sourceName(Source s);

// Name of protection bit `index` (0..12), or nullptr if out of range.
const char *protectionName(uint8_t index);

// One complete reading of the pack. Plain old data: trivially copyable, so it
// can be memcpy'd in and out of the ring without ceremony.
struct BmsSnapshot {
  uint32_t uptimeMs;
  Source   source;
  uint16_t fields;          // bitfield of Field: what this poll actually read
  uint32_t cellsReadMask;   // bit i set = cell i+1 was genuinely read

  float    packVolts;
  float    currentAmps;      // positive = charging
  float    remainingAh;
  float    fullAh;
  uint16_t cycles;
  uint8_t  socPercent;

  uint16_t protection;       // bitfield of Protection
  uint32_t balanceBits;      // bit n set = cell n+1 balancing
  bool     chargeMosOn;
  bool     dischargeMosOn;

  uint8_t  cellCount;        // channels the BMS is configured for
  uint8_t  ntcCount;
  uint16_t cellMilliVolts[kMaxCells];
  float    ntcCelsius[kMaxNtc];

  uint16_t softwareVersion;
  uint16_t mfgYear;
  uint8_t  mfgMonth;
  uint8_t  mfgDay;

  // Filled by computeDerived(). A channel reading 0 mV is a configured-but-
  // unpopulated tap and is excluded from the min/max/spread statistics - on
  // this pack channel 10 is exactly that.
  uint8_t  cellsRead;        // channels actually read this poll
  uint8_t  liveCellCount;    // of those, how many carry voltage
  uint16_t cellMinMv;
  uint16_t cellMaxMv;
  uint16_t cellSpreadMv;
  float    cellSumVolts;

  bool     valid;            // false until a link has filled it in
};

// True if every field group in `mask` was obtained.
inline bool hasFields(const BmsSnapshot &s, uint16_t mask) { return (s.fields & mask) == mask; }

// Recomputes the derived statistics. Only channels flagged in cellsReadMask are
// considered, so a dropped cell frame does not drag the sum or minimum down.
// Safe to call on a zeroed snapshot.
void computeDerived(BmsSnapshot &s);

// True when every field the two links share agrees within tolerance. Used to
// cross-check the UART reading against the CAN reading.
bool snapshotsAgree(const BmsSnapshot &a, const BmsSnapshot &b,
                    float voltTolerance = 0.05f, uint16_t cellToleranceMv = 5);

}  // namespace jbd
