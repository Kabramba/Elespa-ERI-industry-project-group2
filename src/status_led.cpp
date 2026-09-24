#include "status_led.h"

#include <string.h>

// --- platform hookup --------------------------------------------------------
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#define STATUS_ON_TARGET 1
// GPIO48 on the ESP32-S3-DevKitC-1. PIN_NEOPIXEL comes from the variant.
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN PIN_NEOPIXEL
#endif
#else
#include <chrono>
#define STATUS_ON_TARGET 0
#endif

namespace status {
namespace {

uint32_t nowMs() {
#if STATUS_ON_TARGET
  return millis();
#else
  using namespace std::chrono;
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
#endif
}

void noopWrite(uint8_t, uint8_t, uint8_t) {}

#if STATUS_ON_TARGET && STATUS_LED_ENABLED
// neopixelWrite() keeps its RMT handle in a function-local static and is not
// reentrant, which is one reason tick() is single-task only.
void hardwareWrite(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(STATUS_LED_PIN, r, g, b); }
#endif

}  // namespace

// ============================== StatusLed ===================================

void StatusLed::begin(Writer w, uint8_t brightnessPercent) {
  {
    util::LockGuard g(lock_);
    memset(slots_, 0, sizeof(slots_));
    sequence_   = 0;
    activeIdx_  = -1;
    everWrote_  = false;
    last_       = Color{0, 0, 0};
    brightness_ = brightnessPercent > 100 ? 100 : brightnessPercent;
    writer_     = (w != nullptr) ? w : &noopWrite;
  }
  // A WS2812 latches whatever noise it saw at power-on and holds it. Write
  // black immediately so an uninitialised LED never sits there at full tilt.
  writer_(0, 0, 0);
  last_      = Color{0, 0, 0};
  everWrote_ = true;
}

void StatusLed::setBrightness(uint8_t percent) {
  {
    util::LockGuard g(lock_);
    brightness_ = percent > 100 ? 100 : percent;
  }
  everWrote_ = false;  // force the next tick() to re-send at the new scale
}

Token StatusLed::claim(const Pattern &p, uint32_t holdMs, uint32_t now) {
  util::LockGuard g(lock_);

  int idx = -1;
  for (size_t i = 0; i < kMaxClaims; ++i)
    if (!slots_[i].used) {
      idx = static_cast<int>(i);
      break;
    }

  if (idx < 0) {
    // Table full: evict the least important, oldest claim - but never one that
    // outranks the arrival, or an alarm could be displaced by routine traffic.
    int worst = 0;
    for (size_t i = 1; i < kMaxClaims; ++i) {
      const bool lower = slots_[i].pattern.priority < slots_[worst].pattern.priority;
      const bool older = slots_[i].pattern.priority == slots_[worst].pattern.priority &&
                         slots_[i].sequence < slots_[worst].sequence;
      if (lower || older) worst = static_cast<int>(i);
    }
    if (slots_[worst].pattern.priority > p.priority) return kNoToken;
    idx = worst;
  }

  Slot &s       = slots_[idx];
  s.pattern     = p;
  s.expiresAtMs = holdMs ? (now + holdMs) : 0;
  s.sequence    = ++sequence_;
  s.generation  = static_cast<uint16_t>(s.generation + 1);
  if (s.generation == 0) s.generation = 1;  // 0 is reserved so kNoToken stays invalid
  s.used = true;

  return (static_cast<Token>(s.generation) << 16) | static_cast<Token>(idx + 1);
}

void StatusLed::release(Token t) {
  if (t == kNoToken) return;
  const size_t   idx = static_cast<size_t>(t & 0xFFFFu);
  const uint16_t gen = static_cast<uint16_t>(t >> 16);
  if (idx == 0 || idx > kMaxClaims) return;

  util::LockGuard g(lock_);
  Slot          &s = slots_[idx - 1];
  // The generation check stops a stale token from freeing a slot that has since
  // been reused by somebody else.
  if (s.used && s.generation == gen) s.used = false;
}

void StatusLed::releaseAll() {
  util::LockGuard g(lock_);
  for (Slot &s : slots_) s.used = false;
  activeIdx_ = -1;
}

int StatusLed::pickWinner(uint32_t now) const {
  int best = -1;
  for (size_t i = 0; i < kMaxClaims; ++i) {
    const Slot &s = slots_[i];
    if (!s.used) continue;
    if (s.expiresAtMs != 0 && static_cast<int32_t>(now - s.expiresAtMs) >= 0) continue;
    if (best < 0) {
      best = static_cast<int>(i);
      continue;
    }
    const Slot &b      = slots_[best];
    const bool  higher = s.pattern.priority > b.pattern.priority;
    const bool  newer  = s.pattern.priority == b.pattern.priority && s.sequence > b.sequence;
    if (higher || newer) best = static_cast<int>(i);
  }
  return best;
}

void StatusLed::tick(uint32_t now) {
  if (writer_ == nullptr) return;

  Color target{0, 0, 0};
  {
    util::LockGuard g(lock_);
    for (Slot &s : slots_)
      if (s.used && s.expiresAtMs != 0 && static_cast<int32_t>(now - s.expiresAtMs) >= 0)
        s.used = false;

    activeIdx_ = pickWinner(now);
    if (activeIdx_ >= 0) {
      const Pattern &p  = slots_[activeIdx_].pattern;
      bool           on = true;
      if (p.periodMs > 0) {
        const uint32_t phase = now % p.periodMs;
        const uint32_t onFor = static_cast<uint32_t>(p.periodMs) * p.dutyPercent / 100u;
        on                   = phase < onFor;
      }
      if (on) target = p.color;
    }
  }
  // Written outside the lock: neopixelWrite() drives RMT, and doing that with
  // interrupts masked by a critical section invites trouble.
  write(target);
}

void StatusLed::write(Color c) {
  const Color scaled{static_cast<uint8_t>(static_cast<uint16_t>(c.r) * brightness_ / 100u),
                     static_cast<uint8_t>(static_cast<uint16_t>(c.g) * brightness_ / 100u),
                     static_cast<uint8_t>(static_cast<uint16_t>(c.b) * brightness_ / 100u)};
  if (everWrote_ && scaled.r == last_.r && scaled.g == last_.g && scaled.b == last_.b) return;
  last_      = scaled;
  everWrote_ = true;
  writer_(scaled.r, scaled.g, scaled.b);
}

const Pattern *StatusLed::activePattern() const {
  util::LockGuard g(lock_);
  if (activeIdx_ < 0) return nullptr;
  const Slot &s = slots_[activeIdx_];
  return s.used ? &s.pattern : nullptr;
}

size_t StatusLed::activeClaims() const {
  util::LockGuard g(lock_);
  size_t n = 0;
  for (const Slot &s : slots_)
    if (s.used) ++n;
  return n;
}

// =========================== module singleton ===============================

StatusLed &led() {
  static StatusLed instance;
  return instance;
}

void begin(uint8_t brightnessPercent) {
#if STATUS_ON_TARGET && STATUS_LED_ENABLED
  led().begin(&hardwareWrite, brightnessPercent);
#else
  led().begin(&noopWrite, brightnessPercent);
#endif
}

void tick() { led().tick(nowMs()); }
void setBrightness(uint8_t percent) { led().setBrightness(percent); }
void release(Token t) { led().release(t); }

// ============================= named states =================================
namespace {

constexpr Color kRed{255, 0, 0};
constexpr Color kGreen{0, 255, 0};
constexpr Color kBlue{0, 80, 255};
constexpr Color kCyan{0, 255, 200};
constexpr Color kMagenta{255, 0, 200};
constexpr Color kYellow{255, 190, 0};
constexpr Color kOrange{255, 80, 0};
constexpr Color kWhite{255, 255, 255};

Token hold(const Pattern &p) { return led().claim(p, 0, nowMs()); }
Token flash(const Pattern &p, uint32_t ms) { return led().claim(p, ms, nowMs()); }

}  // namespace

// ---- lifecycle ----
Token show_boot() { return flash({kWhite, Priority::kInfo, 0, 100, "boot"}, 600); }
Token show_idle() { return hold({kBlue, Priority::kIdle, 0, 100, "idle"}); }
Token show_links_up() { return flash({kGreen, Priority::kInfo, 0, 100, "links-up"}, 800); }
Token show_link_init_failed() {
  return hold({kRed, Priority::kCritical, 0, 100, "link-init-failed"});
}

// ---- momentary activity ----
Token show_uart_poll() { return flash({kCyan, Priority::kActivity, 0, 100, "uart-poll"}, 60); }
Token show_uart_ok() { return flash({kGreen, Priority::kActivity, 0, 100, "uart-ok"}, 60); }
Token show_can_poll() { return flash({kMagenta, Priority::kActivity, 0, 100, "can-poll"}, 60); }
Token show_can_ok() { return flash({kGreen, Priority::kActivity, 0, 100, "can-ok"}, 60); }
Token show_ring_published() {
  return flash({kWhite, Priority::kActivity, 0, 100, "ring-published"}, 25);
}

// ---- link health ----
Token show_link_degraded() { return hold({kYellow, Priority::kWarning, 1200, 30, "link-degraded"}); }
Token show_can_silent() { return hold({kMagenta, Priority::kWarning, 1200, 30, "can-silent"}); }

// ---- pack state ----
Token show_pack_idle() { return hold({kBlue, Priority::kInfo, 0, 100, "pack-idle"}); }
Token show_pack_charging() { return hold({kGreen, Priority::kInfo, 2000, 50, "charging"}); }
Token show_pack_discharging() { return hold({kBlue, Priority::kInfo, 2000, 50, "discharging"}); }
Token show_pack_balancing() { return hold({kCyan, Priority::kInfo, 1000, 50, "balancing"}); }
Token show_soc_low() { return hold({kOrange, Priority::kWarning, 0, 100, "soc-low"}); }
Token show_soc_critical() { return hold({kRed, Priority::kError, 300, 50, "soc-critical"}); }
Token show_mos_disabled() { return hold({kOrange, Priority::kWarning, 800, 50, "mos-disabled"}); }
Token show_protection_active() {
  return hold({kRed, Priority::kError, 0, 100, "protection-active"});
}

// ---- numbered error states ----
Token show_error_state50() { return hold({kRed, Priority::kError, 500, 50, "E50 uart-timeout"}); }
Token show_error_state51() { return hold({kRed, Priority::kError, 250, 50, "E51 uart-checksum"}); }
Token show_error_state52() { return hold({kMagenta, Priority::kError, 500, 50, "E52 can-no-ack"}); }
Token show_error_state53() { return hold({kRed, Priority::kCritical, 120, 50, "E53 can-bus-off"}); }
Token show_error_state54() { return hold({kMagenta, Priority::kError, 250, 50, "E54 can-crc"}); }
Token show_error_state55() { return hold({kOrange, Priority::kError, 120, 50, "E55 can-tx-stuck"}); }
Token show_error_state56() { return hold({kYellow, Priority::kWarning, 250, 50, "E56 ring-overflow"}); }
Token show_error_state57() { return hold({kYellow, Priority::kWarning, 500, 50, "E57 link-mismatch"}); }
Token show_error_state58() { return hold({kRed, Priority::kError, 0, 100, "E58 bms-protection"}); }
Token show_error_state59() { return hold({kWhite, Priority::kCritical, 100, 50, "E59 internal"}); }

}  // namespace status
