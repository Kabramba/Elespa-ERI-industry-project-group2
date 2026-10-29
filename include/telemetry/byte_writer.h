// byte_writer.h - the narrowest possible view of "somewhere bytes can go".
//
// The transport sinks depend on this instead of on Arduino's Stream, which is
// what lets the entire framing and publishing path - COBS, the envelope, the
// sequence numbering - run under the host test suite with a writer that just
// appends to an array. The Arduino adapter is the only piece that cannot be
// tested on the host, and it is four lines long.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

class IByteWriter {
 public:
  virtual ~IByteWriter() = default;

  // Write `length` bytes. Returns how many were accepted; a short write means
  // the transport is backed up and the caller must treat the frame as lost
  // rather than resend the tail, which would corrupt the stream.
  virtual size_t write(const uint8_t *data, size_t length) = 0;
};

}  // namespace telemetry
