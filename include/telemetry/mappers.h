// mappers.h - from what a device said to what the schema publishes.
//
// The acquisition layer's types (jbd::BmsSnapshot, obd::VehicleSnapshot) are
// shaped by the DEVICES: a JBD reply has 17 channels because that is how the
// BMS is configured, and an ECU snapshot keeps raw PID bytes because which PIDs
// exist varies by vehicle. The schema's types are shaped by the CONSUMERS. The
// two must not be the same structs, or every downstream consumer inherits the
// quirks of whatever happened to be on the bus.
//
// Everything here is pure: no Arduino, no clock of its own, no transport. That
// is what lets the mapping be tested against captured fixtures on the host,
// which matters because a mapping bug is invisible on the wire - the packet is
// perfectly well-formed and simply says the wrong thing.
//
// THE FIELD MASK IS TRANSLATED, NOT ASSUMED. If a source did not read a value,
// the corresponding schema bit stays clear. A mapper never invents a zero.
#pragma once

#include <stdint.h>

#include "bms_types.h"
#include "obd/vehicle_types.h"
#include "telemetry/packets.h"
#include "telemetry/stat_block.h"

namespace telemetry {

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
//
// Returns false if the pack has more configured channels than the schema
// carries (kMaxCells). That is not hypothetical: the bench JBD is configured
// for 17 channels on a 16S pack, so the caller is told rather than quietly
// shipping 16 of 17 voltages. The packet is still filled and still valid -
// cellCount reports the true figure - but a consumer comparing cellCount
// against the voltage array needs to know.
bool toBatteryTelemetry(const jbd::BmsSnapshot &s, BatteryTelemetry &out);

// ---------------------------------------------------------------------------
// Vehicle identity
// ---------------------------------------------------------------------------
// Static for the life of the vehicle, so this is published on change and as an
// occasional keepalive rather than on a clock.
void toIdentityTelemetry(const obd::VehicleSnapshot &v, IdentityTelemetry &out);

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------
//
// The engine's fast channels are sampled far faster than they are transmitted -
// RPM at roughly 8 Hz against a 1 Hz packet - so they are accumulated here and
// summarised into StatBlocks at flush time. Pushing one sweep per acquisition
// cycle and flushing once a second is the intended use:
//
//     acc.noteRequest();
//     if (gotReply) { acc.noteReply(); acc.push(snapshot, nowMs); }
//     if (nowMs - lastFlush >= 1000) {
//       EngineTelemetry e;
//       if (acc.flush(e, nowMs)) publisher.publish(kEngine, kEcu, e);
//     }
//
// A channel that never answered during the window produces emptyStats(): NaN
// with samples == 0, and its field bit stays clear. Nothing reads as zero.
class EngineAccumulator {
 public:
  // Start a fresh window. Call once before the first push, and after a flush
  // (flush does it for you).
  void reset(uint32_t nowMs);

  // Fold one PID sweep in. Only channels whose PID actually answered are
  // accumulated, so a dropped reply narrows the statistics rather than
  // poisoning them with a zero.
  void push(const obd::VehicleSnapshot &v, uint32_t nowMs);

  // Link quality over the window. Carried in the packet because a mean RPM
  // computed from three replies is a different thing from one computed from
  // thirty, and the consumer cannot tell without being told.
  void noteRequest() { ++requests_; }
  void noteReply() { ++replies_; }
  void noteCrcError() { if (crcErrors_ < 0xFF) ++crcErrors_; }

  // Summarise the window and start the next one. False - and `out` untouched -
  // if not a single channel was read, so an empty window is not transmitted as
  // a packet full of NaN.
  bool flush(EngineTelemetry &out, uint32_t nowMs);

  uint16_t samples() const { return sweeps_; }

 private:
  RunningStat rpm_, throttle_, map_, timing_, load_;

  float    shortFuelTrim_ = 0.0f;
  int16_t  intakeDeciC_   = 0;
  int16_t  oilDeciC_      = 0;
  uint8_t  fuelSystem_[2] = {};
  uint16_t fields_        = 0;

  uint16_t requests_  = 0;
  uint16_t replies_   = 0;
  uint8_t  crcErrors_ = 0;
  uint16_t sweeps_    = 0;
  uint32_t startMs_   = 0;
};

// ---------------------------------------------------------------------------
// Status and events
// ---------------------------------------------------------------------------
//
// Discrete state, published ON CHANGE. The edge detection has to live
// somewhere, and it cannot live in a pure function: deciding that a DTC
// "appeared" requires knowing what was there before. StatusTracker holds that
// previous state and nothing else.
//
// A fault that appears and clears between two 1 Hz samples would otherwise
// never be seen at all, which is the whole argument for event packets.
class StatusTracker {
 public:
  // Fold in the latest ECU reading. Returns true if anything changed, in which
  // case `out` is filled and should be published; false means the state is
  // identical to last time and there is nothing to say.
  bool update(const obd::VehicleSnapshot &v, StatusEventTelemetry &out);

  // Same, for the battery side. The two sources write disjoint fields of the
  // same packet type, so a caller may publish either independently.
  bool update(const jbd::BmsSnapshot &s, StatusEventTelemetry &out);

  // Forget the previous state, so the next update() reports everything as new.
  // Use after a link drops: what the device says on reconnection is news.
  void forget();

 private:
  bool     haveEcu_          = false;
  bool     haveBattery_      = false;
  uint32_t prevMonitor_      = 0;
  uint32_t prevMonitorCycle_ = 0;
  uint16_t prevDtc_[kMaxDtcs] = {};
  uint8_t  prevDtcListed_    = 0;
  uint8_t  prevStored_       = 0;
  uint8_t  prevPending_      = 0;
  bool     prevMil_          = false;
  uint16_t prevProtection_   = 0;
  uint8_t  prevMosfet_       = 0;
};

}  // namespace telemetry
