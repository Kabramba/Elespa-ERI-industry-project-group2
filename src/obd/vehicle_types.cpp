#include "obd/vehicle_types.h"

#include <math.h>
#include <string.h>

namespace obd {
namespace {
constexpr uint16_t kNoU16 = 0xFFFF;
}  // namespace

void reset(VehicleSnapshot &s) {
  memset(&s, 0, sizeof(s));
  s.rpm = s.speedKph = s.engineLoadPct = s.throttlePct = NAN;
  s.timingAdvanceDeg = s.intakeAirC = s.oilTempC = s.mapKpa = NAN;
  s.shortFuelTrimPct = NAN;
  s.runTimeSec = s.distanceWithMilKm = kNoU16;
}

bool addSample(VehicleSnapshot &s, uint8_t pid, const uint8_t *data, uint8_t len) {
  if (len > 4) len = 4;

  // Upsert. A snapshot is the latest known value for every PID seen, so a fast
  // poll of a few PIDs refreshes those without discarding the rest.
  PidSample *slot = nullptr;
  for (uint8_t i = 0; i < s.sampleCount && i < kMaxPidSamples; ++i)
    if (s.samples[i].pid == pid) { slot = &s.samples[i]; break; }
  if (slot == nullptr) {
    if (s.sampleCount >= kMaxPidSamples) return false;
    slot = &s.samples[s.sampleCount++];
  }
  PidSample &e = *slot;
  e.pid        = pid;
  e.len        = len;
  memset(e.data, 0, sizeof(e.data));
  if (data != nullptr && len) memcpy(e.data, data, len);
  return true;
}

const PidSample *findSample(const VehicleSnapshot &s, uint8_t pid) {
  for (uint8_t i = 0; i < s.sampleCount && i < kMaxPidSamples; ++i)
    if (s.samples[i].pid == pid) return &s.samples[i];
  return nullptr;
}

bool hasPid(const VehicleSnapshot &s, uint8_t pid) { return findSample(s, pid) != nullptr; }

void decodeAll(VehicleSnapshot &s) {
  // Formulas from docs/activa-can.md s6.6.
  for (uint8_t i = 0; i < s.sampleCount && i < kMaxPidSamples; ++i) {
    const PidSample &e = s.samples[i];
    const uint8_t    A = (e.len > 0) ? e.data[0] : 0;
    const uint8_t    B = (e.len > 1) ? e.data[1] : 0;
    switch (e.pid) {
      case 0x01:
        s.milOn          = (A & 0x80) != 0;
        s.storedDtcCount = static_cast<uint8_t>(A & 0x7F);
        break;
      case 0x04: s.engineLoadPct    = A * 100.0f / 255.0f; break;
      case 0x06: s.shortFuelTrimPct = A * 100.0f / 128.0f - 100.0f; break;
      case 0x0B: s.mapKpa           = static_cast<float>(A); break;
      case 0x0C: s.rpm              = (256.0f * A + B) / 4.0f; break;
      case 0x0D: s.speedKph         = static_cast<float>(A); break;
      case 0x0E: s.timingAdvanceDeg = A / 2.0f - 64.0f; break;
      case 0x0F: s.intakeAirC       = static_cast<float>(A) - 40.0f; break;
      case 0x11: s.throttlePct      = A * 100.0f / 255.0f; break;
      case 0x1F: s.runTimeSec       = static_cast<uint16_t>(256u * A + B); break;
      case 0x21: s.distanceWithMilKm = static_cast<uint16_t>(256u * A + B); break;
      case 0x5C: s.oilTempC         = static_cast<float>(A) - 40.0f; break;
      default: break;  // kept raw in samples[]; no formula is better than a guess
    }
  }
}

}  // namespace obd
