// packet_queue.h - the hand-off between the task that produces packets and the
// task that puts them on the wire.
//
// WHY A QUEUE AND NOT THE SNAPSHOT RING. util::SnapshotRing holds the latest
// reading of one thing and lets a consumer re-read it; its job is state. This
// holds a stream of distinct packets that must each be transmitted exactly
// once; its job is transfer. Reading the same packet twice would duplicate it on
// the wire, and holding only the newest would drop types entirely - a status
// event published between two battery packets would never be sent.
//
// OVERFLOW is newest-wins, matching the ring: when full, the OLDEST queued
// packet is discarded. On a telemetry link the freshest data is the useful
// data, and a consumer detects the gap from the header's sequence numbers
// anyway. Discards are counted, so falling behind is visible rather than
// silent.
//
// CONCURRENCY: one producer, one consumer, possibly on different cores. The
// critical sections are a memcpy of one packet.
//
// STORAGE: fixed slots in .bss, like everything else here. No allocation, no
// flash, gone on power cycle. N slots cost N * kMaxPacketBytes bytes - at the
// default depth of 8 that is about 2.6 kB, which an ESP32-S3 with 320 kB of
// SRAM will not notice.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform_lock.h"
#include "telemetry/framing.h"

namespace telemetry {

template <size_t N>
class PacketQueue {
  static_assert(N >= 2, "a queue of one slot cannot buffer anything");

 public:
  // Copy a packet in. False if it was rejected outright (nothing to copy, or
  // longer than any legal packet); a queue-full discard still returns true,
  // because the caller's packet WAS accepted - an older one made room for it.
  bool push(const uint8_t *packet, size_t length) {
    if (packet == nullptr || length == 0 || length > kMaxPacketBytes) {
      util::LockGuard g(lock_);
      ++rejected_;
      return false;
    }

    util::LockGuard g(lock_);
    if (count_ == N) {
      // Drop the oldest to make room.
      tail_ = (tail_ + 1) % N;
      --count_;
      ++dropped_;
    }
    memcpy(slot_[head_].bytes, packet, length);
    slot_[head_].length = length;
    head_               = (head_ + 1) % N;
    ++count_;
    ++pushed_;
    return true;
  }

  // Copy the oldest packet out and remove it. Returns its length, or 0 when the
  // queue is empty or `outMax` is too small to take it - in the latter case the
  // packet is LEFT IN PLACE, so a caller with an undersized buffer loses
  // nothing and can retry with a bigger one.
  size_t pop(uint8_t *out, size_t outMax) {
    if (out == nullptr) return 0;
    util::LockGuard g(lock_);
    if (count_ == 0) return 0;
    const size_t len = slot_[tail_].length;
    if (len > outMax) return 0;
    memcpy(out, slot_[tail_].bytes, len);
    tail_ = (tail_ + 1) % N;
    --count_;
    return len;
  }

  bool   empty() const { util::LockGuard g(lock_); return count_ == 0; }
  size_t count() const { util::LockGuard g(lock_); return count_; }
  size_t capacity() const { return N; }

  uint32_t pushedCount() const { util::LockGuard g(lock_); return pushed_; }
  uint32_t droppedCount() const { util::LockGuard g(lock_); return dropped_; }
  uint32_t rejectedCount() const { util::LockGuard g(lock_); return rejected_; }

  void clear() {
    util::LockGuard g(lock_);
    head_ = tail_ = count_ = 0;
  }

 private:
  struct Slot {
    size_t  length = 0;
    uint8_t bytes[kMaxPacketBytes] = {};
  };

  Slot   slot_[N] = {};
  size_t head_    = 0;
  size_t tail_    = 0;
  size_t count_   = 0;

  uint32_t pushed_   = 0;
  uint32_t dropped_  = 0;
  uint32_t rejected_ = 0;

  util::SpinLock lock_;
};

}  // namespace telemetry
