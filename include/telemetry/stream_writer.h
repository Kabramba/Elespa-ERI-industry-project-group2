// stream_writer.h - the Arduino adapter, and the only part of the transport
// that cannot be tested on the host.
//
// Kept deliberately trivial for that reason: everything with logic in it -
// COBS, the envelope, sequence numbering, the queue - sits behind IByteWriter
// and runs under the host suite. What is left here is a forwarding call.
#pragma once

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "telemetry/byte_writer.h"

namespace telemetry {

class StreamByteWriter final : public IByteWriter {
 public:
  explicit StreamByteWriter(Stream &s) : s_(s) {}

  size_t write(const uint8_t *data, size_t length) override {
    // Stream::write returns a short count when the TX buffer cannot take the
    // rest. CobsFrameSink treats that as a lost packet rather than resending
    // the tail, which would put a torn frame on the wire.
    return s_.write(data, length);
  }

 private:
  Stream &s_;
};

}  // namespace telemetry

#endif
