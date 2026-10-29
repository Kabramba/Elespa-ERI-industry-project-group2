// publisher.h - the API application code uses to emit a telemetry packet.
//
// This is the seam the whole schema exists for. An application fills in one of
// the seven payload structs from packets.h and hands it over; everything about
// the envelope - sequence numbering, the producer clock, the CRC, the byte
// layout - is handled here, so no application ever builds a header by hand.
//
//     telemetry::BatteryTelemetry b = {};
//     fill(b);
//     publisher.publish(telemetry::PacketType::kBattery,
//                       telemetry::SourceId::kBatteryA, b);
//
// SEQUENCE NUMBERS ARE PER SOURCE, because that is what the header says and
// because it is what makes them useful: a consumer watching SourceId::kEcu
// detects a gap in the ECU's stream without the battery's packets perturbing
// the count. They wrap at 16 bits, which at 1 Hz is 18 hours - a consumer must
// compare with wrapping arithmetic, not with `>`.
//
// THREAD SAFETY. One Publisher may be shared by several producer tasks: the
// encode buffer and the sequence counters are guarded. What is NOT serialised
// is the sink behind it, so a sink that is itself shared needs its own
// protection - the queue in packet_queue.h has it.
//
// WHERE THE PACKET GOES is entirely the sink's business. Publisher neither
// knows nor cares whether the bytes end up on a UART, in a queue, or in a test
// vector.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "platform_lock.h"
#include "telemetry/byte_writer.h"
#include "telemetry/framing.h"
#include "telemetry/packet.h"
#include "telemetry/packet_queue.h"

namespace telemetry {

// A destination for one fully encoded packet. The envelope is built and the CRC
// computed before a sink ever sees the bytes: a sink moves them, it does not
// interpret them.
class IPacketSink {
 public:
  virtual ~IPacketSink() = default;
  virtual const char *name() const = 0;

  // False means "this packet did not get through" - a full queue, a backed-up
  // transport. The publisher counts it and moves on; telemetry is lossy by
  // nature and blocking a producer to retry would make the next reading late as
  // well as this one lost.
  virtual bool onPacket(const uint8_t *packet, size_t length) = 0;
};

// The producer's clock, in milliseconds since boot. Injected rather than
// calling millis() directly, which is what lets the publish path be tested on
// the host with a clock the test drives.
using ClockFn = uint32_t (*)();

class Publisher {
 public:
  Publisher(IPacketSink &sink, ClockFn clock) : sink_(sink), clock_(clock) {}

  Publisher(const Publisher &)            = delete;
  Publisher &operator=(const Publisher &) = delete;

  // The typed form, and the one to use: sizeof(T) cannot disagree with the
  // struct being passed.
  template <typename T>
  bool publish(PacketType type, SourceId source, const T &payload, uint8_t flags = 0) {
    return publishRaw(type, source, &payload, static_cast<uint16_t>(sizeof(T)), flags);
  }

  bool publishRaw(PacketType type, SourceId source, const void *payload,
                  uint16_t payloadBytes, uint8_t flags = 0);

  // Next sequence number this source will use. Diagnostics only.
  uint16_t sequenceFor(SourceId source) const;

  uint32_t publishedCount() const { util::LockGuard g(lock_); return published_; }
  uint32_t refusedCount() const { util::LockGuard g(lock_); return refused_; }   // sink said no
  uint32_t encodeErrors() const { util::LockGuard g(lock_); return encodeErr_; } // payload too big

 private:
  IPacketSink &sink_;
  ClockFn      clock_;

  // No encode buffer here on purpose: publishRaw() encodes on the stack so the
  // lock is held only across the sequence counter and the statistics.
  uint16_t seq_[kSourceCount] = {};

  uint32_t published_ = 0;
  uint32_t refused_   = 0;
  uint32_t encodeErr_ = 0;

  mutable util::SpinLock lock_;
};

// ---------------------------------------------------------------------------
// FanOut - send every packet to several sinks.
// ---------------------------------------------------------------------------
//
// The usual arrangement during bring-up: the real transport plus a readable
// debug sink, so what the board is emitting can be watched without a decoder.
// Reports success only if EVERY sink accepted the packet, so a silently
// failing transport cannot hide behind a working debug one.
template <size_t N>
class FanOutSink final : public IPacketSink {
 public:
  const char *name() const override { return "fan-out"; }

  // False if the table is full.
  bool add(IPacketSink *s) {
    if (s == nullptr || count_ >= N) return false;
    sink_[count_++] = s;
    return true;
  }

  bool onPacket(const uint8_t *packet, size_t length) override {
    bool all = count_ > 0;
    for (size_t i = 0; i < count_; ++i) {
      if (!sink_[i]->onPacket(packet, length)) all = false;
    }
    return all;
  }

  size_t count() const { return count_; }

 private:
  IPacketSink *sink_[N] = {};
  size_t       count_   = 0;
};

// ---------------------------------------------------------------------------
// QueueSink - the inter-task hop.
// ---------------------------------------------------------------------------
template <size_t N>
class QueueSink final : public IPacketSink {
 public:
  explicit QueueSink(PacketQueue<N> &q) : q_(q) {}
  const char *name() const override { return "queue"; }
  bool onPacket(const uint8_t *packet, size_t length) override {
    return q_.push(packet, length);
  }

 private:
  PacketQueue<N> &q_;
};

// ---------------------------------------------------------------------------
// CobsFrameSink - the one that actually puts packets on a UART.
// ---------------------------------------------------------------------------
//
// COBS-encodes the packet and writes body + delimiter in a SINGLE write() call.
// That matters: two writes could be interleaved with another task's output and
// produce a frame that no reader can decode. If the writer cannot take the
// whole frame the packet is reported as lost rather than partially sent, since
// a truncated frame costs the receiver a resynchronisation.
class CobsFrameSink final : public IPacketSink {
 public:
  explicit CobsFrameSink(IByteWriter &writer) : writer_(writer) {}
  const char *name() const override { return "cobs"; }
  bool onPacket(const uint8_t *packet, size_t length) override;

  uint32_t framesWritten() const { return written_; }
  uint32_t shortWrites() const { return shortWrites_; }

 private:
  IByteWriter &writer_;
  uint8_t      frame_[kMaxFrameBytes] = {};
  uint32_t     written_               = 0;
  uint32_t     shortWrites_           = 0;
};

// ---------------------------------------------------------------------------
// HexLinePacketSink - packets as text, so they can share a port with text.
// ---------------------------------------------------------------------------
//
// Writes one line per packet: "T " then the packet in lowercase hex, then a
// newline. Twice the bytes of COBS and no use at all on a dedicated link - but
// it is the only form that can be interleaved with the line-oriented
// diagnostics the live apps already print. On a vehicle, where there is one USB
// cable and the dashboard is watching it, that matters more than the overhead.
//
// The newline is the delimiter, so no byte stuffing is needed: hex cannot
// contain one.
class HexLinePacketSink final : public IPacketSink {
 public:
  explicit HexLinePacketSink(IByteWriter &writer) : writer_(writer) {}
  const char *name() const override { return "hex-line"; }
  bool onPacket(const uint8_t *packet, size_t length) override;

  uint32_t linesWritten() const { return written_; }
  uint32_t shortWrites() const { return shortWrites_; }

 private:
  // "T " + two characters per byte + newline.
  char     line_[2 + kMaxPacketBytes * 2 + 1] = {};
  IByteWriter &writer_;
  uint32_t     written_     = 0;
  uint32_t     shortWrites_ = 0;
};

}  // namespace telemetry
