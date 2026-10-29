#include "telemetry/mappers.h"

#include <math.h>
#include <string.h>

namespace telemetry {
namespace {

// The JBD's protection word and the schema's are bit-for-bit identical, which
// lets the mapping be a straight copy. That is a coincidence of two independent
// definitions, so it is asserted rather than trusted: renumber either side and
// the build breaks, instead of every fault being silently relabelled - a
// short-circuit arriving as an over-temperature is the kind of bug that gets
// believed.
static_assert(static_cast<uint16_t>(jbd::kCellOv) == static_cast<uint16_t>(kProtCellOv), "");
static_assert(static_cast<uint16_t>(jbd::kCellUv) == static_cast<uint16_t>(kProtCellUv), "");
static_assert(static_cast<uint16_t>(jbd::kPackOv) == static_cast<uint16_t>(kProtPackOv), "");
static_assert(static_cast<uint16_t>(jbd::kPackUv) == static_cast<uint16_t>(kProtPackUv), "");
static_assert(static_cast<uint16_t>(jbd::kChgOt) == static_cast<uint16_t>(kProtChgOt), "");
static_assert(static_cast<uint16_t>(jbd::kChgUt) == static_cast<uint16_t>(kProtChgUt), "");
static_assert(static_cast<uint16_t>(jbd::kDsgOt) == static_cast<uint16_t>(kProtDsgOt), "");
static_assert(static_cast<uint16_t>(jbd::kDsgUt) == static_cast<uint16_t>(kProtDsgUt), "");
static_assert(static_cast<uint16_t>(jbd::kChgOc) == static_cast<uint16_t>(kProtChgOc), "");
static_assert(static_cast<uint16_t>(jbd::kDsgOc) == static_cast<uint16_t>(kProtDsgOc), "");
static_assert(static_cast<uint16_t>(jbd::kShort) == static_cast<uint16_t>(kProtShort), "");
static_assert(static_cast<uint16_t>(jbd::kAfeErr) == static_cast<uint16_t>(kProtAfeError), "");
static_assert(static_cast<uint16_t>(jbd::kMosSwLock) == static_cast<uint16_t>(kProtMosLock), "");

// Celsius to tenths, saturating. A sensor reading off the scale must not wrap
// round to a plausible temperature.
int16_t deciC(float c) {
  if (isnan(c)) return 0;
  const float t = c * 10.0f;
  if (t > 32767.0f) return 32767;
  if (t < -32768.0f) return -32768;
  return static_cast<int16_t>(t);
}

// A PID that did not answer leaves its decoded field at NaN.
bool present(float v) { return !isnan(v); }

// One raw PID answer assembled big-endian, as OBD-II sends it.
bool rawU32(const obd::VehicleSnapshot &v, uint8_t pid, uint32_t &out) {
  const obd::PidSample *s = obd::findSample(v, pid);
  if (s == nullptr) return false;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < s->len && i < 4; ++i) acc = (acc << 8) | s->data[i];
  out = acc;
  return true;
}

void copyFixed(char *dst, size_t dstSize, const char *src) {
  memset(dst, 0, dstSize);
  if (src == nullptr) return;
  const size_t n = strnlen(src, dstSize - 1);
  memcpy(dst, src, n);
}

}  // namespace

// ============================== battery ====================================

bool toBatteryTelemetry(const jbd::BmsSnapshot &s, BatteryTelemetry &out) {
  memset(&out, 0, sizeof(out));

  out.vendor    = static_cast<uint8_t>(BatteryVendor::kJbd);
  out.chemistry = static_cast<uint8_t>(CellChemistry::kUnknown);  // not reported

  // The current is a fast channel in the schema. A single snapshot is one
  // sample, so it is published as a one-sample window; a caller that
  // accumulates across a window overwrites this with its own StatBlock.
  if (jbd::hasFields(s, jbd::kFieldPack)) {
    RunningStat r;
    r.reset(s.uptimeMs);
    r.push(s.currentAmps, s.uptimeMs);
    out.currentAmps = r.snapshot();

    out.packVolts   = s.packVolts;
    out.remainingAh = s.remainingAh;
    out.fields |= kBatPack | kBatCurrent;
  } else {
    out.currentAmps = emptyStats();
  }

  if (jbd::hasFields(s, jbd::kFieldCapacity)) {
    out.fullAh     = s.fullAh;
    out.cycleCount = s.cycles;
    out.socPercent = s.socPercent;
    out.fields |= kBatCapacity;
  }

  if (jbd::hasFields(s, jbd::kFieldProtect)) {
    out.protectionFlags = s.protection;
    out.balancingMask   = s.balanceBits;
    out.fields |= kBatProtection | kBatBalance;
  }

  if (jbd::hasFields(s, jbd::kFieldMos)) {
    out.mosfetState = static_cast<uint8_t>((s.chargeMosOn ? kMosCharge : 0) |
                                          (s.dischargeMosOn ? kMosDischarge : 0));
    out.fields |= kBatMosfet;
  }

  // cellCount is reported truthfully even when it exceeds what the voltage
  // array can carry, so a consumer can detect the truncation.
  if (jbd::hasFields(s, jbd::kFieldConfig)) out.cellCount = s.cellCount;

  bool allChannelsFitted = true;

  if (jbd::hasFields(s, jbd::kFieldCells)) {
    const uint8_t configured = s.cellCount;
    const uint8_t carried    = (configured < kMaxCells) ? configured : kMaxCells;
    allChannelsFitted        = (configured <= kMaxCells);

    uint16_t minMv = 0xFFFF, maxMv = 0;
    uint8_t  minIx = 0, maxIx = 0;
    bool     any = false;

    for (uint8_t i = 0; i < carried; ++i) {
      const bool read = (s.cellsReadMask & (1u << i)) != 0;
      if (!read) continue;
      const uint16_t mv   = s.cellMilliVolts[i];
      out.cellMilliVolts[i] = mv;

      // A channel reading exactly 0 mV is a configured-but-unpopulated tap -
      // channel 10 on this pack - so it is marked absent and excluded from the
      // spread, which is otherwise dominated by it.
      if (mv == 0) continue;
      out.cellPresentMask |= (1u << i);
      ++out.liveCellCount;
      any = true;
      if (mv < minMv) { minMv = mv; minIx = i; }
      if (mv > maxMv) { maxMv = mv; maxIx = i; }
    }

    if (any) {
      out.cellMinMv    = minMv;
      out.cellMaxMv    = maxMv;
      out.cellSpreadMv = static_cast<uint16_t>(maxMv - minMv);
      out.cellMinIndex = minIx;
      out.cellMaxIndex = maxIx;
    }
    out.fields |= kBatCells;
  }

  if (jbd::hasFields(s, jbd::kFieldTemps)) {
    const uint8_t n = (s.ntcCount < kMaxTemps) ? s.ntcCount : kMaxTemps;
    out.tempCount   = n;
    int16_t lo = 32767, hi = -32768;
    for (uint8_t i = 0; i < n; ++i) {
      if (isnan(s.ntcCelsius[i])) continue;
      out.tempDeciC[i] = deciC(s.ntcCelsius[i]);
      out.tempPresentMask |= static_cast<uint8_t>(1u << i);
      if (out.tempDeciC[i] < lo) lo = out.tempDeciC[i];
      if (out.tempDeciC[i] > hi) hi = out.tempDeciC[i];
    }
    if (out.tempPresentMask != 0) {
      out.tempMinDeciC = lo;
      out.tempMaxDeciC = hi;
    }
    out.fields |= kBatTemps;
  }

  return allChannelsFitted;
}

// ============================== identity ===================================

void toIdentityTelemetry(const obd::VehicleSnapshot &v, IdentityTelemetry &out) {
  memset(&out, 0, sizeof(out));

  copyFixed(out.vin, sizeof(out.vin), v.vin);
  copyFixed(out.ecuName, sizeof(out.ecuName), v.ecuName);
  copyFixed(out.calibrationId, sizeof(out.calibrationId), v.calibrationId);

  out.cvn                   = v.cvn;
  out.ecuBitrate            = v.bitrate;
  out.ecuExtendedAddressing = v.extendedAddressing ? 1 : 0;

  // PID 1C verbatim. This ECU answers 0x25, which is outside ISO 15031-5 - the
  // raw byte is carried rather than mapped to a standard it does not match.
  uint32_t raw = 0;
  if (rawU32(v, 0x1C, raw)) out.obdStandardRaw = static_cast<uint8_t>(raw & 0xFF);

  if (v.vin[0] != '\0') out.fields |= kIdVehicle;
  if (v.ecuName[0] != '\0' || v.calibrationId[0] != '\0' || v.cvn != 0) {
    out.fields |= kIdEcu;
  }
}

// =============================== engine ====================================

void EngineAccumulator::reset(uint32_t nowMs) {
  rpm_.reset(nowMs);
  throttle_.reset(nowMs);
  map_.reset(nowMs);
  timing_.reset(nowMs);
  load_.reset(nowMs);

  shortFuelTrim_ = 0.0f;
  intakeDeciC_   = 0;
  oilDeciC_      = 0;
  fuelSystem_[0] = fuelSystem_[1] = 0;
  fields_        = 0;
  requests_      = 0;
  replies_       = 0;
  crcErrors_     = 0;
  sweeps_        = 0;
  startMs_       = nowMs;
}

void EngineAccumulator::push(const obd::VehicleSnapshot &v, uint32_t nowMs) {
  ++sweeps_;

  if (present(v.rpm))              { rpm_.push(v.rpm, nowMs);              fields_ |= kEngRpm; }
  if (present(v.throttlePct))      { throttle_.push(v.throttlePct, nowMs); fields_ |= kEngThrottle; }
  if (present(v.mapKpa))           { map_.push(v.mapKpa, nowMs);           fields_ |= kEngMap; }
  if (present(v.timingAdvanceDeg)) { timing_.push(v.timingAdvanceDeg, nowMs); fields_ |= kEngTiming; }
  if (present(v.engineLoadPct))    { load_.push(v.engineLoadPct, nowMs);   fields_ |= kEngLoad; }

  // Slow channels: last value wins, no statistics worth the bytes.
  if (present(v.shortFuelTrimPct)) { shortFuelTrim_ = v.shortFuelTrimPct; fields_ |= kEngFuelTrim; }
  if (present(v.intakeAirC))       { intakeDeciC_ = deciC(v.intakeAirC);  fields_ |= kEngIntakeTemp; }
  if (present(v.oilTempC))         { oilDeciC_ = deciC(v.oilTempC);       fields_ |= kEngOilTemp; }

  const obd::PidSample *fs = obd::findSample(v, 0x03);
  if (fs != nullptr && fs->len >= 1) {
    fuelSystem_[0] = fs->data[0];
    fuelSystem_[1] = (fs->len >= 2) ? fs->data[1] : 0;
    fields_ |= kEngFuelSystem;
  }
}

bool EngineAccumulator::flush(EngineTelemetry &out, uint32_t nowMs) {
  if (fields_ == 0) {
    // Nothing answered all window. Publishing a packet of NaN would tell a
    // consumer less than publishing nothing, and costs it a decode.
    reset(nowMs);
    return false;
  }

  memset(&out, 0, sizeof(out));

  out.rpm              = rpm_.empty() ? emptyStats() : rpm_.snapshot();
  out.throttlePercent  = throttle_.empty() ? emptyStats() : throttle_.snapshot();
  out.mapKpa           = map_.empty() ? emptyStats() : map_.snapshot();
  out.timingAdvanceDeg = timing_.empty() ? emptyStats() : timing_.snapshot();
  out.loadPercent      = load_.empty() ? emptyStats() : load_.snapshot();

  out.shortFuelTrimPercent = shortFuelTrim_;
  out.intakeAirDeciC       = intakeDeciC_;
  out.oilTempDeciC         = oilDeciC_;
  out.fuelSystemStatus[0]  = fuelSystem_[0];
  out.fuelSystemStatus[1]  = fuelSystem_[1];
  out.windowMs             = nowMs - startMs_;
  out.requests             = requests_;
  out.replies              = replies_;
  out.crcErrors            = crcErrors_;
  out.fields               = fields_;

  reset(nowMs);
  return true;
}

// ============================ status and events ============================

void StatusTracker::forget() {
  haveEcu_     = false;
  haveBattery_ = false;
}

bool StatusTracker::update(const obd::VehicleSnapshot &v, StatusEventTelemetry &out) {
  uint32_t monitor = 0, monitorCycle = 0;
  const bool haveMonitor      = rawU32(v, 0x01, monitor);
  const bool haveMonitorCycle = rawU32(v, 0x41, monitorCycle);

  const uint8_t listed = (v.storedDtcCount < kMaxDtcs) ? v.storedDtcCount
                                                       : static_cast<uint8_t>(kMaxDtcs);

  uint16_t events = 0;

  if (!haveEcu_) {
    // First reading: everything is news, but it is not an "edge" - report the
    // state without claiming a transition that was never observed.
    events = 0;
  } else {
    if (v.milOn != prevMil_) events |= kEvMilChanged;

    // A DTC that is present now and was not before, and vice versa. Compared as
    // sets, because an ECU is under no obligation to keep the order stable.
    for (uint8_t i = 0; i < listed; ++i) {
      bool wasThere = false;
      for (uint8_t j = 0; j < prevDtcListed_; ++j) {
        if (prevDtc_[j] == v.dtc[i]) { wasThere = true; break; }
      }
      if (!wasThere) { events |= kEvDtcAppeared; break; }
    }
    for (uint8_t j = 0; j < prevDtcListed_; ++j) {
      bool stillThere = false;
      for (uint8_t i = 0; i < listed; ++i) {
        if (v.dtc[i] == prevDtc_[j]) { stillThere = true; break; }
      }
      if (!stillThere) { events |= kEvDtcCleared; break; }
    }
  }

  const bool changed = !haveEcu_ || events != 0 ||
                       monitor != prevMonitor_ || monitorCycle != prevMonitorCycle_ ||
                       listed != prevDtcListed_ ||
                       v.storedDtcCount != prevStored_ ||
                       v.pendingDtcCount != prevPending_;

  // Remember first: the state is now the previous state whether or not we
  // publish, otherwise an unpublished change is lost and never reported.
  haveEcu_          = true;
  prevMonitor_      = monitor;
  prevMonitorCycle_ = monitorCycle;
  prevDtcListed_    = listed;
  prevStored_       = v.storedDtcCount;
  prevPending_      = v.pendingDtcCount;
  prevMil_          = v.milOn;
  for (uint8_t i = 0; i < listed; ++i) prevDtc_[i] = v.dtc[i];

  if (!changed) return false;

  memset(&out, 0, sizeof(out));
  out.ecuMonitorStatus    = monitor;
  out.ecuMonitorThisCycle = monitorCycle;
  for (uint8_t i = 0; i < listed; ++i) out.dtc[i] = v.dtc[i];
  out.dtcListed       = listed;
  out.storedDtcCount  = v.storedDtcCount;
  out.pendingDtcCount = v.pendingDtcCount;
  out.milOn           = v.milOn ? 1 : 0;
  out.eventMask       = events;
  out.fields          = kStEcuDtc;
  if (haveMonitor || haveMonitorCycle) out.fields |= kStEcuMonitors;

  uint32_t raw = 0;
  if (rawU32(v, 0x1C, raw)) out.obdStandardRaw = static_cast<uint8_t>(raw & 0xFF);

  return true;
}

bool StatusTracker::update(const jbd::BmsSnapshot &s, StatusEventTelemetry &out) {
  const bool haveProtect = jbd::hasFields(s, jbd::kFieldProtect);
  const bool haveMos     = jbd::hasFields(s, jbd::kFieldMos);
  if (!haveProtect && !haveMos) return false;  // nothing was read; say nothing

  const uint16_t protection = haveProtect ? s.protection : prevProtection_;
  const uint8_t  mosfet =
      haveMos ? static_cast<uint8_t>((s.chargeMosOn ? kMosCharge : 0) |
                                     (s.dischargeMosOn ? kMosDischarge : 0))
              : prevMosfet_;

  uint16_t events = 0;
  if (haveBattery_) {
    // Only a NEWLY raised bit is a fault event; one that cleared is not.
    if ((protection & ~prevProtection_) != 0) events |= kEvBatteryFault;
    if (mosfet != prevMosfet_) events |= kEvMosfetChanged;
  }

  const bool changed =
      !haveBattery_ || events != 0 || protection != prevProtection_ || mosfet != prevMosfet_;

  haveBattery_    = true;
  prevProtection_ = protection;
  prevMosfet_     = mosfet;

  if (!changed) return false;

  memset(&out, 0, sizeof(out));
  out.batteryProtectionFlags = protection;
  out.batteryMosfetState     = mosfet;
  out.eventMask              = events;
  out.fields                 = kStBattery;
  return true;
}

}  // namespace telemetry
