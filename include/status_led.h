// status_led.h - the on-board WS2812 as a prioritised status indicator.
//
// WHY A MODULE: the LED is a single shared resource that several unrelated
// parts of the firmware want to drive. Left unmanaged the last writer wins, so
// a routine activity blink can paint over a bus-off alarm. Here every request
// is a CLAIM with a priority; the highest-priority live claim is what shows.
//
// PRIORITY: higher wins. Equal priorities go to the most recent claim. A claim
// lasts until it is released, or until holdMs elapses if one was given - which
// is what makes momentary flashes fire-and-forget.
//
// MUTUAL EXCLUSION: claim() and release() are safe from any task on either
// core. tick() must be called from ONE task only (the main loop): it owns the
// RMT peripheral, and the Arduino core's neopixelWrite() is itself not
// reentrant. The lock is deliberately released before the LED is written, since
// driving RMT inside a critical section with interrupts masked is a bad idea.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "platform_lock.h"

namespace status {

enum class Priority : uint8_t {
  kIdle     = 0,  // nothing happening
  kInfo     = 1,  // steady-state information
  kActivity = 2,  // momentary "something just happened" blinks
  kWarning  = 3,  // degraded but working
  kError    = 4,  // a subsystem has failed
  kCritical = 5,  // unsafe or unrecoverable
};

struct Color {
  uint8_t r, g, b;
};

struct Pattern {
  Color       color;
  Priority    priority;
  uint16_t    periodMs;     // 0 = solid
  uint8_t     dutyPercent;  // on-fraction of the period when blinking
  const char *name;
};

using Token = uint32_t;
constexpr Token kNoToken = 0;

// Hardware hook. On the ESP32 this drives the WS2812; host tests substitute a
// capture stub, which is what makes the arbitration testable off-target.
using Writer = void (*)(uint8_t r, uint8_t g, uint8_t b);

constexpr size_t kMaxClaims = 8;

class StatusLed {
 public:
  StatusLed() = default;
  // Holds a lock, so it must never be copied or moved. Stated outright rather
  // than left as an accident of the member's type.
  StatusLed(const StatusLed &) = delete;
  StatusLed &operator=(const StatusLed &) = delete;
  StatusLed(StatusLed &&) = delete;
  StatusLed &operator=(StatusLed &&) = delete;

  void    begin(Writer w, uint8_t brightnessPercent);
  void    setBrightness(uint8_t percent);  // clamped to 0..100
  uint8_t brightness() const { return brightness_; }

  // Claim the LED. holdMs == 0 means "until released".
  // Returns kNoToken only if the writer was never installed.
  Token claim(const Pattern &p, uint32_t holdMs, uint32_t nowMs);
  void  release(Token t);
  void  releaseAll();

  // Expire timed claims, pick the winner, drive the LED. One task only.
  void tick(uint32_t nowMs);

  const Pattern *activePattern() const;
  size_t         activeClaims() const;
  Color          lastWritten() const { return last_; }

 private:
  struct Slot {
    Pattern  pattern;
    uint32_t expiresAtMs;  // 0 = never
    uint32_t sequence;     // tie-break among equal priorities; higher is newer
    uint16_t generation;   // invalidates stale tokens when a slot is reused
    bool     used;
  };

  int  pickWinner(uint32_t nowMs) const;  // -1 if none
  void write(Color c);

  Slot     slots_[kMaxClaims] = {};
  uint32_t sequence_          = 0;
  uint8_t  brightness_        = STATUS_LED_BRIGHTNESS_PERCENT;
  Writer   writer_            = nullptr;
  Color    last_              = {0, 0, 0};

  // Guards the slot table. Never held while the LED is written.
  mutable util::SpinLock lock_;
  bool     everWrote_         = false;
  int      activeIdx_         = -1;
};

// ---- module-level singleton ------------------------------------------------
StatusLed &led();
void       begin(uint8_t brightnessPercent = STATUS_LED_BRIGHTNESS_PERCENT);
void       tick();                       // call once per loop()
void       setBrightness(uint8_t percent);
void       release(Token t);

// ---- named states ----------------------------------------------------------
// Steady states are held until released; momentary ones expire on their own.

// Lifecycle
Token show_boot();                 // white, brief
Token show_idle();                 // dim blue, held
Token show_links_up();             // green, brief
Token show_link_init_failed();     // red solid, held

// Momentary activity - fire and forget
Token show_uart_poll();            // cyan flash
Token show_uart_ok();              // green flash
Token show_can_poll();             // magenta flash
Token show_can_ok();               // green flash
Token show_ring_published();       // white pinprick

// Link health - held
Token show_link_degraded();        // yellow, slow blink
Token show_can_silent();           // magenta, slow blink

// Pack state - held
Token show_pack_idle();            // dim blue
Token show_pack_charging();        // green, slow breath-blink
Token show_pack_discharging();     // blue, slow blink
Token show_pack_balancing();       // cyan, slow blink
Token show_soc_low();              // orange
Token show_soc_critical();         // red, fast blink
Token show_mos_disabled();         // orange, slow blink
Token show_protection_active();    // red, held

// ---- numbered error states -------------------------------------------------
// Distinct colour + blink rate each, so a fault is identifiable across a room.
Token show_error_state50();  // UART: no reply / timeout          red    500 ms
Token show_error_state51();  // UART: checksum or framing         red    250 ms
Token show_error_state52();  // CAN: remote frame not ACKed       magenta 500 ms
Token show_error_state53();  // CAN: bus-off                      red    120 ms
Token show_error_state54();  // CAN: CRC mismatch                 magenta 250 ms
Token show_error_state55();  // CAN: TX stuck / queue jammed      orange 120 ms
Token show_error_state56();  // ring buffer overflowing           yellow 250 ms
Token show_error_state57();  // UART and CAN disagree             yellow 500 ms
Token show_error_state58();  // BMS reports a protection fault    red solid
Token show_error_state59();  // internal / unclassified           white  100 ms

}  // namespace status
