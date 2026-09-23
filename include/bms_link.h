// bms_link.h - a source of pack telemetry.
//
// Both transports implement this, so main.cpp does not care which is which and
// a third (RS485, Bluetooth, a simulator for bench work) drops in unchanged.
#pragma once

#include <stdint.h>

#include "bms_types.h"

namespace jbd {

struct LinkStats {
  uint32_t attempts;
  uint32_t successes;
  uint32_t failures;
  uint32_t crcErrors;
  // CAN only; zero on UART.
  uint32_t busErrors;
  uint32_t txErrorCounter;
};

class IBmsLink {
 public:
  virtual ~IBmsLink() = default;

  virtual const char *name() const = 0;

  // Bring the peripheral up. False means the link is unusable.
  virtual bool begin() = 0;

  // Attempt one complete reading. Returns true and fills `out` on success.
  // Must not block for more than a few hundred milliseconds.
  virtual bool poll(BmsSnapshot &out) = 0;

  virtual void stats(LinkStats &out) const = 0;
};

}  // namespace jbd
