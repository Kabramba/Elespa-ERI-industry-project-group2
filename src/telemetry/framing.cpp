#include "telemetry/framing.h"

namespace telemetry {

// kMaxPayloadBytes must really be the maximum. If a new packet type outgrows
// DynamicsTelemetry this fails at compile time, rather than every frame of the
// new type being silently rejected as an overrun at run time.
static_assert(sizeof(IdentityTelemetry) <= kMaxPayloadBytes, "identity outgrew the frame buffer");
static_assert(sizeof(EngineTelemetry) <= kMaxPayloadBytes, "engine outgrew the frame buffer");
static_assert(sizeof(MotorTelemetry) <= kMaxPayloadBytes, "motor outgrew the frame buffer");
static_assert(sizeof(DynamicsTelemetry) <= kMaxPayloadBytes, "dynamics outgrew the frame buffer");
static_assert(sizeof(NavigationTelemetry) <= kMaxPayloadBytes, "navigation outgrew the frame buffer");
static_assert(sizeof(BatteryTelemetry) <= kMaxPayloadBytes, "battery outgrew the frame buffer");
static_assert(sizeof(StatusEventTelemetry) <= kMaxPayloadBytes, "status outgrew the frame buffer");

size_t cobsEncode(const uint8_t *in, size_t n, uint8_t *out, size_t outMax) {
  if (out == nullptr || outMax == 0) return 0;
  if (in == nullptr && n > 0) return 0;

  size_t  codeIdx  = 0;  // slot reserved for the current run's length
  size_t  writeIdx = 1;  // next data slot
  uint8_t code     = 1;  // run length + 1; 0xFF means "254 bytes, no zero"

  for (size_t readIdx = 0; readIdx < n; ++readIdx) {
    if (in[readIdx] != 0) {
      if (writeIdx >= outMax) return 0;
      out[writeIdx++] = in[readIdx];
      ++code;
      if (code != 0xFF) continue;
    }
    // Either the byte was a zero - which the code byte now stands in for, so it
    // is not written - or the run hit its 254-byte ceiling.
    out[codeIdx] = code;
    codeIdx      = writeIdx++;
    if (writeIdx > outMax) return 0;  // the slot just reserved must exist
    code = 1;
  }

  out[codeIdx] = code;
  return writeIdx;
}

size_t cobsDecode(const uint8_t *in, size_t n, uint8_t *out, size_t outMax) {
  if (in == nullptr || out == nullptr) return 0;

  size_t readIdx  = 0;
  size_t writeIdx = 0;

  while (readIdx < n) {
    const uint8_t code = in[readIdx++];
    if (code == 0) return 0;  // a zero inside a body: this is not a COBS frame

    for (uint8_t i = 1; i < code; ++i) {
      if (readIdx >= n) return 0;        // run claims more bytes than the frame has
      if (writeIdx >= outMax) return 0;  // decoded output would not fit
      out[writeIdx++] = in[readIdx++];
    }

    // A code of 0xFF is a full run with no zero after it. Otherwise the run was
    // terminated by a zero - except at the very end of the frame, where the
    // zero was the delimiter and has already been stripped.
    if (code != 0xFF && readIdx < n) {
      if (writeIdx >= outMax) return 0;
      out[writeIdx++] = 0;
    }
  }

  return writeIdx;
}

void FrameReader::reset() {
  rawLen_     = 0;
  discarding_ = false;
}

size_t FrameReader::push(uint8_t byte) {
  if (byte != kDelimiter) {
    if (discarding_) return 0;  // already over length; wait for the delimiter
    if (rawLen_ >= sizeof(raw_)) {
      // Longer than any legal frame. Stop accumulating and count it once.
      ++overruns_;
      discarding_ = true;
      rawLen_     = 0;
      return 0;
    }
    raw_[rawLen_++] = byte;
    return 0;
  }

  // Delimiter: end of frame.
  if (discarding_) {
    reset();
    return 0;
  }

  const size_t len = rawLen_;
  rawLen_          = 0;

  // Back-to-back delimiters, or a stream that opens with one. Not an error -
  // it is how a writer idles the line and how a reader finds its first edge.
  if (len == 0) return 0;

  const size_t decoded = cobsDecode(raw_, len, packet_, sizeof(packet_));
  if (decoded == 0) {
    ++framing_;
    return 0;
  }

  if (!verify(packet_, decoded)) {
    ++crc_;
    return 0;
  }

  ++accepted_;
  return decoded;
}

}  // namespace telemetry
