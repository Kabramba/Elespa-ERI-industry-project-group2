#include "telemetry/publisher.h"

namespace telemetry {

bool Publisher::publishRaw(PacketType type, SourceId source, const void *payload,
                           uint16_t payloadBytes, uint8_t flags) {
  // An unknown SourceId would index off the end of the counter table. Clamp to
  // slot 0 rather than refusing the packet: losing a per-source sequence is a
  // far smaller problem than dropping telemetry, and the header still carries
  // the source byte verbatim so a consumer can see what happened.
  const size_t slot     = static_cast<size_t>(source);
  const size_t safeSlot = (slot < kSourceCount) ? slot : 0;

  uint16_t seq = 0;
  {
    util::LockGuard g(lock_);
    seq = seq_[safeSlot]++;
  }

  // Encoded on the stack, unlocked. kMaxPacketBytes is 324 bytes, which is fine
  // on an 8 kB Arduino loop task and is the price of never holding a spinlock
  // across a transport write - a UART write can block for milliseconds, and
  // stalling the other core to do it is exactly the mistake status_led.cpp
  // avoids by releasing its lock before touching the RMT peripheral.
  uint8_t      frame[kMaxPacketBytes];
  const size_t n = encode(type, source, seq, clock_ != nullptr ? clock_() : 0, flags,
                          payload, payloadBytes, frame, sizeof(frame));
  if (n == 0) {
    util::LockGuard g(lock_);
    ++encodeErr_;
    return false;
  }

  const bool ok = sink_.onPacket(frame, n);

  util::LockGuard g(lock_);
  if (ok) {
    ++published_;
  } else {
    ++refused_;
  }
  return ok;
}

uint16_t Publisher::sequenceFor(SourceId source) const {
  const size_t slot = static_cast<size_t>(source);
  util::LockGuard g(lock_);
  return seq_[(slot < kSourceCount) ? slot : 0];
}

bool CobsFrameSink::onPacket(const uint8_t *packet, size_t length) {
  // Leave room for the delimiter, which must go out in the same write.
  const size_t e = cobsEncode(packet, length, frame_, sizeof(frame_) - 1);
  if (e == 0) return false;
  frame_[e] = kDelimiter;

  const size_t want = e + 1;
  if (writer_.write(frame_, want) != want) {
    ++shortWrites_;
    return false;
  }
  ++written_;
  return true;
}

bool HexLinePacketSink::onPacket(const uint8_t *packet, size_t length) {
  if (packet == nullptr || length == 0 || length > kMaxPacketBytes) return false;

  static const char kHex[] = "0123456789abcdef";
  size_t            n      = 0;
  line_[n++]               = 'T';
  line_[n++]               = ' ';
  for (size_t i = 0; i < length; ++i) {
    line_[n++] = kHex[packet[i] >> 4];
    line_[n++] = kHex[packet[i] & 0x0F];
  }
  line_[n++] = '\n';

  // One write, like the COBS sink: a line torn in half by another task's output
  // is a line no reader can parse.
  if (writer_.write(reinterpret_cast<const uint8_t *>(line_), n) != n) {
    ++shortWrites_;
    return false;
  }
  ++written_;
  return true;
}

}  // namespace telemetry
