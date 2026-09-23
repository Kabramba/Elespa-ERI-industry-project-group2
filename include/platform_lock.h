// platform_lock.h - the one place that knows whether we are on FreeRTOS.
//
// The ESP32-S3 is dual core, so the ring must be safe against a producer task
// on one core and a consumer on the other. On the host the same interface maps
// onto std::mutex so the ring can be unit-tested natively.
#pragma once

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include "freertos/FreeRTOS.h"

namespace jbd {
class SpinLock {
 public:
  void lock() const { portENTER_CRITICAL(&mux_); }
  void unlock() const { portEXIT_CRITICAL(&mux_); }
 private:
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
}  // namespace jbd

#else

#include <mutex>

namespace jbd {
class SpinLock {
 public:
  void lock() const { m_.lock(); }
  void unlock() const { m_.unlock(); }
 private:
  mutable std::mutex m_;
};
}  // namespace jbd

#endif

namespace jbd {
// RAII guard. Critical sections here are a few microseconds - one struct copy.
class LockGuard {
 public:
  explicit LockGuard(const SpinLock &l) : l_(l) { l_.lock(); }
  ~LockGuard() { l_.unlock(); }
  LockGuard(const LockGuard &) = delete;
  LockGuard &operator=(const LockGuard &) = delete;
 private:
  const SpinLock &l_;
};
}  // namespace jbd
