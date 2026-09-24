// snapshot_ring.h - the IPC buffer between a producer and whatever consumes it.
//
// Generic over the payload type, because the battery and the vehicle produce
// different shapes of data and must not share a buffer. T needs to be trivially
// copyable and to have a `valid` member (latestRef() uses it to report "nothing
// published yet").
//
// STORAGE: an instance declared `static` or at namespace scope lives in .bss/
// .data, i.e. plain RAM. It is reinitialised by the C runtime on every boot and
// is never written to flash, NVS or EEPROM. It is deliberately NOT tagged
// RTC_DATA_ATTR or RTC_NOINIT_ATTR, which would survive resets and deep sleep.
// Power cycle the board and the history is gone. That is the intent.
//
// OVERFLOW: fixed capacity, newest-wins. When full, publishing discards the
// oldest entry and bumps overwrittenCount() so a consumer can tell it fell
// behind rather than silently missing samples.
//
// CONCURRENCY: one producer, any number of consumers, and they may sit on
// different cores. Two read paths are offered:
//
//   latest(out) / history(dst, max)
//       Copy out under a short critical section. Always coherent, never torn.
//       Costs one struct copy. Use this unless the copy actually hurts.
//
//   latestRef()
//       Zero-copy. Returns a pointer into a double buffer that the producer
//       never writes while it is published; publish() fills the other half and
//       then flips an atomic index. The pointee stays valid until TWO further
//       publishes, so read what you need promptly and do not store the pointer
//       across a poll cycle. Dereferencing it later is a data race.
//
// Every consumer-facing method is const. A consumer holding a `const BmsRing&`
// cannot mutate the buffer.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <atomic>

#include "platform_lock.h"

namespace util {

template <typename T, size_t N>
class SnapshotRing {
  static_assert(N >= 2, "ring needs at least two slots");

 public:
  // ---- producer side ----------------------------------------------------
  void publish(const T &s) {
    // Zero-copy path first: fill the half nobody is reading, then flip.
    const uint8_t cur = frontIdx_.load(std::memory_order_relaxed);
    const uint8_t nxt = static_cast<uint8_t>(cur ^ 1);
    front_[nxt] = s;
    frontIdx_.store(nxt, std::memory_order_release);

    // Then the history ring.
    LockGuard g(lock_);
    if (count_ == N) ++overwritten_;
    ring_[head_] = s;
    head_ = (head_ + 1) % N;
    if (count_ < N) ++count_;
    ++published_;
  }

  // ---- consumer side (all const) ----------------------------------------

  // Newest snapshot, copied. False if nothing has been published yet.
  bool latest(T &out) const {
    LockGuard g(lock_);
    if (count_ == 0) return false;
    out = ring_[(head_ + N - 1) % N];
    return true;
  }

  // Newest snapshot, no copy. nullptr until the first publish.
  // Valid until two further publishes - see the header comment.
  const T *latestRef() const {
    const T *p = &front_[frontIdx_.load(std::memory_order_acquire)];
    return p->valid ? p : nullptr;
  }

  // Up to `max` of the most recent snapshots, written oldest-first.
  // Returns how many were written.
  size_t history(T *dst, size_t max) const {
    if (dst == nullptr || max == 0) return 0;
    LockGuard g(lock_);
    const size_t n     = (count_ < max) ? count_ : max;
    const size_t start = (head_ + N - n) % N;
    for (size_t i = 0; i < n; ++i) dst[i] = ring_[(start + i) % N];
    return n;
  }

  size_t   capacity() const { return N; }
  size_t   count() const { LockGuard g(lock_); return count_; }
  uint32_t publishedCount() const { LockGuard g(lock_); return published_; }
  uint32_t overwrittenCount() const { LockGuard g(lock_); return overwritten_; }

  // ---- producer-side reset ----------------------------------------------
  void clear() {
    LockGuard g(lock_);
    head_ = 0;
    count_ = 0;
    published_ = 0;
    overwritten_ = 0;
    memset(ring_, 0, sizeof(ring_));
    memset(front_, 0, sizeof(front_));
    frontIdx_.store(0, std::memory_order_release);
  }

 private:
  T ring_[N] = {};
  size_t      head_ = 0;
  size_t      count_ = 0;
  uint32_t    published_ = 0;
  uint32_t    overwritten_ = 0;
  SpinLock    lock_;

  T                    front_[2] = {};
  std::atomic<uint8_t> frontIdx_{0};
};

}  // namespace util
