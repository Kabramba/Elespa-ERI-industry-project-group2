// battery_packet_sink.h - the bridge from the BMS acquisition layer to the
// published schema.
//
// Deliberately an IBmsSink, because that is the extension point main_jbd.cpp
// already documents: "adding a consumer means adding an IBmsSink, and neither
// touches this file beyond one array entry". Publishing packets is just another
// consumer of pack telemetry, alongside the ring and the serial printout.
//
// WHAT IT PUBLISHES. One kBattery packet per snapshot, and a kStatusEvent only
// when the discrete state actually changed - the protection word or the MOSFET
// states. A status packet on every poll would be almost entirely repetition,
// and would bury the one transition that mattered.
//
// THE CURRENT IS ACCUMULATED ACROSS SNAPSHOTS. Pack current is the fast channel
// in BatteryTelemetry, and a single poll is one sample. Snapshots are folded
// into a RunningStat and the StatBlock is attached at publish time, so a
// consumer polling at 0.5 Hz still receives the min, max and spread of what
// happened in between rather than one arbitrary instant.
#pragma once

#include <stdint.h>

#include "bms_sink.h"
#include "telemetry/mappers.h"
#include "telemetry/packets.h"
#include "telemetry/publisher.h"
#include "telemetry/stat_block.h"

namespace telemetry {

class BatteryPacketSink final : public jbd::IBmsSink {
 public:
  // `source` distinguishes the two packs: the JBD is kBatteryA, the Daly
  // kBatteryB. The consumer needs that to tell them apart, and the sequence
  // numbers are per source for the same reason.
  BatteryPacketSink(Publisher &publisher, SourceId source, BatteryVendor vendor)
      : pub_(publisher), source_(source), vendor_(vendor) {}

  const char *name() const override { return "packet"; }

  void onSnapshot(const jbd::BmsSnapshot &s) override {
    if (jbd::hasFields(s, jbd::kFieldPack)) {
      if (!haveCurrent_) {
        current_.reset(s.uptimeMs);
        haveCurrent_ = true;
      }
      current_.push(s.currentAmps, s.uptimeMs);
    }

    BatteryTelemetry b = {};
    const bool       allChannelsFitted = toBatteryTelemetry(s, b);
    b.vendor                           = static_cast<uint8_t>(vendor_);

    // Replace the mapper's single-sample block with the accumulated window.
    if (haveCurrent_ && !current_.empty()) b.currentAmps = current_.snapshot();

    // A poll that read only some field groups is partial, and the flag says so
    // without a consumer having to work it out from the field mask.
    uint8_t flags = 0;
    const uint16_t everything = jbd::kFieldPack | jbd::kFieldCapacity | jbd::kFieldProtect |
                                jbd::kFieldMos | jbd::kFieldConfig | jbd::kFieldTemps |
                                jbd::kFieldCells;
    if ((s.fields & everything) != everything) flags |= kFlagPartial;
    if (!allChannelsFitted) {
      // More configured channels than the schema carries. The packet is still
      // correct for the channels it holds, and cellCount reports the true
      // figure, but a consumer should not assume the array is complete.
      flags |= kFlagPartial;
      ++truncated_;
    }

    if (pub_.publish(PacketType::kBattery, source_, b, flags)) {
      ++published_;
      current_.reset(s.uptimeMs);  // the window closes with the packet
    }

    StatusEventTelemetry st = {};
    if (status_.update(s, st)) {
      if (pub_.publish(PacketType::kStatusEvent, source_, st, 0)) ++events_;
    }
  }

  // Forget the previous discrete state, so the next snapshot reports it afresh.
  // Call after a link drops: what the BMS says on reconnection is news.
  void linkLost() {
    status_.forget();
    haveCurrent_ = false;
  }

  uint32_t packetsPublished() const { return published_; }
  uint32_t eventsPublished() const { return events_; }
  uint32_t truncatedSnapshots() const { return truncated_; }

 private:
  Publisher    &pub_;
  SourceId      source_;
  BatteryVendor vendor_;

  StatusTracker status_;
  RunningStat   current_;
  bool          haveCurrent_ = false;

  uint32_t published_ = 0;
  uint32_t events_    = 0;
  uint32_t truncated_ = 0;
};

}  // namespace telemetry
