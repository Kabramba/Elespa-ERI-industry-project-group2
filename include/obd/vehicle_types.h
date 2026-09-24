// vehicle_types.h - one complete reading of a vehicle's ECU.
//
// Deliberately free of Arduino and TWAI headers so the decode logic is
// host-testable, exactly like bms_types.h.
//
// Unlike a BMS, an ECU's field set is not fixed: which PIDs exist varies by
// vehicle, so raw samples are kept alongside the decoded conveniences. Nothing
// the ECU said is thrown away just because we lack a formula for it.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace obd {

constexpr size_t kMaxPidSamples = 48;
constexpr size_t kMaxDtcs       = 8;

// One PID's raw answer, exactly as the ECU sent it.
struct PidSample {
  uint8_t pid;
  uint8_t len;      // valid bytes in data
  uint8_t data[4];  // standard Service 01 PIDs are at most 4 bytes
};

struct VehicleSnapshot {
  uint32_t uptimeMs;
  bool     valid;

  // Link, recorded so a consumer knows how this was obtained.
  uint32_t bitrate;
  bool     extendedAddressing;

  // Identity, from Service 09. Static for the life of the vehicle.
  char     vin[20];
  char     calibrationId[20];
  char     ecuName[24];
  uint32_t cvn;

  // Faults.
  bool     milOn;
  uint8_t  storedDtcCount;
  uint8_t  pendingDtcCount;
  uint16_t dtc[kMaxDtcs];

  // Raw PID answers. The record of what was actually said.
  uint8_t   sampleCount;
  PidSample samples[kMaxPidSamples];

  // Decoded conveniences. NaN / 0xFFFF means "that PID was not supported or
  // did not answer" - check hasPid() rather than trusting a zero.
  float    rpm;
  float    speedKph;
  float    engineLoadPct;
  float    throttlePct;
  float    timingAdvanceDeg;
  float    intakeAirC;
  float    oilTempC;
  float    mapKpa;
  float    shortFuelTrimPct;
  uint16_t runTimeSec;
  uint16_t distanceWithMilKm;
};

// Record one raw PID answer. False if the table is full.
bool addSample(VehicleSnapshot &s, uint8_t pid, const uint8_t *data, uint8_t len);

// Find a recorded sample, or nullptr.
const PidSample *findSample(const VehicleSnapshot &s, uint8_t pid);
bool             hasPid(const VehicleSnapshot &s, uint8_t pid);

// Fill the decoded fields from whatever raw samples are present. Safe on a
// zeroed snapshot; anything absent stays at its not-present marker.
void decodeAll(VehicleSnapshot &s);

// Clear to the not-present state. Call before filling a fresh sweep.
void reset(VehicleSnapshot &s);

}  // namespace obd
