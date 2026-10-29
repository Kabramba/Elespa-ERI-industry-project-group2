// framing.h - putting a packet on a byte stream and getting it back off.
//
// WHY FRAMING IS NEEDED AT ALL. packet.h gives every packet a magic number, but
// a magic number cannot delimit a binary stream: the payload is arbitrary bytes,
// so a cell voltage pair or a float's mantissa will eventually contain 0x5A 0xE1
// by pure chance. Searching for the magic would then lock onto the middle of a
// packet and stay out of step. A reader needs a byte that CANNOT occur inside a
// frame.
//
// COBS (Consistent Overhead Byte Stuffing) provides exactly that. It removes
// every zero byte from the encoded body, so 0x00 becomes an unambiguous frame
// delimiter:
//
//   - overhead is one byte per 254 bytes of payload, plus the delimiter. The
//     largest packet here (16 + 308 = 324 bytes) costs 3 extra bytes, 0.9 %.
//   - the overhead is CONSTANT, unlike the byte-doubling schemes (SLIP, PPP)
//     whose worst case is 2x and whose buffer sizing is therefore a guess.
//   - resynchronisation after any corruption costs at most one frame: discard
//     bytes until the next 0x00 and you are aligned again, by construction.
//
// So the wire format is:
//
//     [ COBS( header | payload ) ] [ 0x00 ]
//
// and the CRC inside the header still does the integrity check. COBS only
// establishes where a frame starts and ends; it detects nothing.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "telemetry/packet.h"
#include "telemetry/packets.h"

namespace telemetry {

// The largest payload of any packet type. DynamicsTelemetry is the biggest -
// the IMU and wheel channels are the highest-rate source by far, so it carries
// the most StatBlocks. static_assert'd against every type in framing.cpp, so
// adding a bigger packet breaks the build rather than silently truncating.
constexpr size_t kMaxPayloadBytes = sizeof(DynamicsTelemetry);
constexpr size_t kMaxPacketBytes  = kHeaderBytes + kMaxPayloadBytes;

// COBS adds one code byte per 254-byte run, plus one leading code byte.
constexpr size_t cobsMaxEncoded(size_t n) { return n + (n / 254) + 1; }

// A fully framed packet, delimiter included. Size your buffers with this.
constexpr size_t kMaxFrameBytes = cobsMaxEncoded(kMaxPacketBytes) + 1;

// The frame delimiter. Cannot appear in a COBS-encoded body.
constexpr uint8_t kDelimiter = 0x00;

// ---------------------------------------------------------------------------
// COBS, as free functions so they can be tested without any transport.
// ---------------------------------------------------------------------------

// Encode `n` bytes into `out`, WITHOUT the trailing delimiter (the caller adds
// it, because a writer usually wants to append it to the same write).
// Returns bytes written, or 0 if `outMax` is too small - never a partial write.
size_t cobsEncode(const uint8_t *in, size_t n, uint8_t *out, size_t outMax);

// Decode one frame's body (delimiter already stripped).
// Returns bytes written, or 0 on malformed input or insufficient room. A zero
// return is unambiguous here: a valid packet is never shorter than a header.
size_t cobsDecode(const uint8_t *in, size_t n, uint8_t *out, size_t outMax);

// ---------------------------------------------------------------------------
// FrameReader - the receive side, fed one byte at a time.
// ---------------------------------------------------------------------------
//
// Deliberately byte-at-a-time rather than buffer-at-a-time: a UART delivers
// bytes whenever it feels like it, and a reader that needs whole frames handed
// to it just pushes the reassembly problem back onto its caller.
//
// Only packets that are well-framed AND pass verify() (magic, schema version,
// declared length, CRC) are reported. Everything else increments a counter, and
// the counters are the point: on a link you cannot see, the ratio of
// framingErrors to crcErrors tells you whether you are losing bytes or
// corrupting them, which are different faults with different fixes.
class FrameReader {
 public:
  // Feed one received byte. Returns the length of a complete, verified packet -
  // now readable via packet() until the next push() - or 0 if there is nothing
  // yet. Call it for every byte and act on a non-zero return.
  size_t push(uint8_t byte);

  // Valid only immediately after push() returned non-zero.
  const uint8_t *packet() const { return packet_; }

  // Discard any partial frame. The next delimiter re-establishes alignment.
  void reset();

  uint32_t framesAccepted() const { return accepted_; }
  uint32_t framingErrors() const { return framing_; }   // COBS could not decode
  uint32_t crcErrors() const { return crc_; }           // decoded, failed verify()
  uint32_t overruns() const { return overruns_; }       // frame longer than legal

 private:
  uint8_t raw_[kMaxFrameBytes] = {};  // the encoded body, delimiter excluded
  size_t  rawLen_              = 0;
  bool    discarding_          = false;  // overran; skip to the next delimiter

  uint8_t packet_[kMaxPacketBytes] = {};

  uint32_t accepted_ = 0;
  uint32_t framing_  = 0;
  uint32_t crc_      = 0;
  uint32_t overruns_ = 0;
};

}  // namespace telemetry
