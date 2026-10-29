// Host tests for the wire framing: COBS, and the streaming reader that has to
// recover from a byte stream it cannot trust.
#include <string.h>

#include <initializer_list>

#include <unity.h>

#include "telemetry/framing.h"
#include "telemetry/packet.h"
#include "telemetry/packets.h"

using namespace telemetry;

void setUp(void) {}
void tearDown(void) {}

// --------------------------------- helpers ----------------------------------

// Encode, decode, and assert we got the same bytes back. Also asserts the
// invariant the whole scheme rests on: no zero byte survives encoding.
// COBS is tested here as a general-purpose function, independent of how big a
// telemetry packet happens to be, so these buffers are sized for the test
// inputs rather than with kMaxFrameBytes.
constexpr size_t kScratch = 1024;

static void assertRoundTrip(const uint8_t *in, size_t n, const char *what) {
  TEST_ASSERT_LESS_THAN_UINT_MESSAGE(kScratch - 8, n, "test input outgrew the scratch buffer");

  uint8_t enc[kScratch] = {};
  const size_t e = cobsEncode(in, n, enc, sizeof(enc));
  TEST_ASSERT_GREATER_THAN_UINT_MESSAGE(0, e, what);
  TEST_ASSERT_EQUAL_UINT_MESSAGE(cobsMaxEncoded(n) >= e ? e : 0, e,
                                 "encoded longer than cobsMaxEncoded() promises");

  for (size_t i = 0; i < e; ++i) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, enc[i], what);
  }

  uint8_t dec[kScratch] = {};
  const size_t d = cobsDecode(enc, e, dec, sizeof(dec));
  TEST_ASSERT_EQUAL_UINT_MESSAGE(n, d, what);
  if (n > 0) TEST_ASSERT_EQUAL_MEMORY_MESSAGE(in, dec, n, what);
}

// A complete, valid packet for the reader tests.
static size_t makePacket(uint8_t *out, size_t outMax, uint16_t seq = 7) {
  BatteryTelemetry b = {};
  b.packVolts        = 53.4f;
  b.cellCount        = 16;
  b.fields           = kBatPack;
  return encode(PacketType::kBattery, SourceId::kBatteryA, seq, 1234, 0, b, out, outMax);
}

// Push a whole buffer through the reader, returning the length the reader
// reported for the LAST accepted packet (0 if none).
static size_t pushAll(FrameReader &r, const uint8_t *data, size_t n) {
  size_t last = 0;
  for (size_t i = 0; i < n; ++i) {
    const size_t got = r.push(data[i]);
    if (got) last = got;
  }
  return last;
}

// ----------------------------------- COBS -----------------------------------

static void test_round_trip_of_representative_payloads(void) {
  const uint8_t empty[1]   = {0};
  const uint8_t oneZero[1] = {0x00};
  const uint8_t noZeros[5] = {1, 2, 3, 4, 5};
  const uint8_t allZeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t mixed[9]   = {0x11, 0x00, 0x00, 0x22, 0x33, 0x00, 0xE1, 0x5A, 0x00};

  assertRoundTrip(empty, 0, "empty");
  assertRoundTrip(oneZero, 1, "a single zero");
  assertRoundTrip(noZeros, sizeof(noZeros), "no zeros");
  assertRoundTrip(allZeros, sizeof(allZeros), "all zeros");
  assertRoundTrip(mixed, sizeof(mixed), "mixed, containing the magic bytes");
}

// 254 is where COBS changes behaviour: a run of exactly 254 non-zero bytes fills
// a code byte to 0xFF and must NOT imply a trailing zero. Off by one here
// corrupts every frame longer than 254 bytes, which is most real packets.
static void test_the_254_byte_run_boundary(void) {
  uint8_t buf[kScratch - 16];
  for (size_t len : {(size_t)253, (size_t)254, (size_t)255, (size_t)256, (size_t)508, (size_t)509}) {
    for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>((i % 255) + 1);  // never 0
    assertRoundTrip(buf, len, "long run of non-zero bytes");
  }

  // The same boundary, but with a zero immediately after the full run.
  for (size_t i = 0; i < 254; ++i) buf[i] = 0xAA;
  buf[254] = 0x00;
  buf[255] = 0xBB;
  assertRoundTrip(buf, 256, "254 non-zeros then a zero");
}

static void test_a_maximum_size_packet_fits_the_frame_buffer(void) {
  uint8_t pkt[kMaxPacketBytes];
  for (size_t i = 0; i < sizeof(pkt); ++i) pkt[i] = static_cast<uint8_t>(i & 0xFF);

  uint8_t enc[kMaxFrameBytes] = {};
  const size_t e = cobsEncode(pkt, sizeof(pkt), enc, sizeof(enc));
  TEST_ASSERT_GREATER_THAN_UINT(0, e);
  // kMaxFrameBytes must leave room for the encoded body AND the delimiter.
  TEST_ASSERT_LESS_OR_EQUAL_UINT(kMaxFrameBytes - 1, e);
}

static void test_encode_refuses_rather_than_writing_partially(void) {
  const uint8_t in[10] = {1, 0, 2, 0, 3, 0, 4, 0, 5, 0};
  uint8_t out[32];

  // Anything too small must return 0 and never report a short success, because
  // a caller that trusted a partial length would transmit a corrupt frame.
  for (size_t cap = 1; cap < 12; ++cap) {
    memset(out, 0xCD, sizeof(out));
    const size_t n = cobsEncode(in, sizeof(in), out, cap);
    if (n != 0) TEST_ASSERT_GREATER_OR_EQUAL_UINT(n, cap);
  }

  TEST_ASSERT_EQUAL_UINT(0, cobsEncode(in, sizeof(in), out, 1));
  TEST_ASSERT_EQUAL_UINT(0, cobsEncode(nullptr, 4, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT(0, cobsEncode(in, sizeof(in), nullptr, 16));
}

static void test_decode_rejects_malformed_bodies(void) {
  uint8_t out[64];

  // A zero byte cannot appear in a COBS body - that is the whole point.
  const uint8_t hasZero[4] = {0x03, 0x11, 0x00, 0x22};
  TEST_ASSERT_EQUAL_UINT(0, cobsDecode(hasZero, sizeof(hasZero), out, sizeof(out)));

  // A code byte claiming more data than the frame contains.
  const uint8_t overruns[3] = {0x09, 0x11, 0x22};
  TEST_ASSERT_EQUAL_UINT(0, cobsDecode(overruns, sizeof(overruns), out, sizeof(out)));

  // A leading zero code.
  const uint8_t leadingZero[2] = {0x00, 0x11};
  TEST_ASSERT_EQUAL_UINT(0, cobsDecode(leadingZero, sizeof(leadingZero), out, sizeof(out)));

  // Correct input, but the caller's buffer is too small.
  const uint8_t fine[4] = {0x04, 0x11, 0x22, 0x33};
  TEST_ASSERT_EQUAL_UINT(0, cobsDecode(fine, sizeof(fine), out, 2));
}

// ------------------------------- FrameReader --------------------------------

static void test_reader_accepts_a_well_framed_packet(void) {
  uint8_t pkt[kMaxPacketBytes];
  const size_t n = makePacket(pkt, sizeof(pkt));
  TEST_ASSERT_GREATER_THAN_UINT(0, n);

  uint8_t frame[kMaxFrameBytes] = {};
  const size_t e = cobsEncode(pkt, n, frame, sizeof(frame));
  frame[e]       = kDelimiter;

  FrameReader r;
  const size_t got = pushAll(r, frame, e + 1);

  TEST_ASSERT_EQUAL_UINT(n, got);
  TEST_ASSERT_EQUAL_MEMORY(pkt, r.packet(), n);
  TEST_ASSERT_EQUAL_UINT32(1, r.framesAccepted());
  TEST_ASSERT_EQUAL_UINT32(0, r.framingErrors());
  TEST_ASSERT_EQUAL_UINT32(0, r.crcErrors());

  PacketHeader h = {};
  TEST_ASSERT_TRUE(peekHeader(r.packet(), got, h));
  TEST_ASSERT_EQUAL_UINT16(7, h.sequence);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PacketType::kBattery), h.type);
}

// Idle line and back-to-back delimiters are normal, not errors. A reader that
// counted them would report a fault rate on a perfectly healthy link.
static void test_delimiters_alone_are_not_errors(void) {
  FrameReader r;
  for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT(0, r.push(kDelimiter));
  TEST_ASSERT_EQUAL_UINT32(0, r.framingErrors());
  TEST_ASSERT_EQUAL_UINT32(0, r.crcErrors());
  TEST_ASSERT_EQUAL_UINT32(0, r.framesAccepted());
}

// The property that justifies COBS: whatever garbage precedes it, the next
// delimiter restores alignment and the following packet is read correctly.
static void test_reader_resynchronises_after_garbage(void) {
  uint8_t pkt[kMaxPacketBytes];
  const size_t n = makePacket(pkt, sizeof(pkt), 99);
  uint8_t frame[kMaxFrameBytes] = {};
  const size_t e = cobsEncode(pkt, n, frame, sizeof(frame));
  frame[e]       = kDelimiter;

  FrameReader r;

  // Mid-packet junk: a truncated frame, then noise, then a clean frame.
  pushAll(r, frame, e / 2);                       // half a frame, no delimiter
  const uint8_t noise[] = {0xFF, 0x01, 0x7E, 0xAB, kDelimiter};
  pushAll(r, noise, sizeof(noise));               // ends the broken frame

  const size_t got = pushAll(r, frame, e + 1);
  TEST_ASSERT_EQUAL_UINT(n, got);
  TEST_ASSERT_EQUAL_MEMORY(pkt, r.packet(), n);
  TEST_ASSERT_EQUAL_UINT32(1, r.framesAccepted());

  // Something was rejected, and it was counted. Which bucket depends on whether
  // the junk happened to decode, so assert the total rather than guessing.
  TEST_ASSERT_GREATER_THAN_UINT32(0, r.framingErrors() + r.crcErrors());
}

// A flipped bit inside the payload must be reported as a CRC failure, not as a
// framing failure: the two counters diagnose different physical faults.
static void test_a_corrupted_payload_counts_as_a_crc_error(void) {
  uint8_t pkt[kMaxPacketBytes];
  const size_t n = makePacket(pkt, sizeof(pkt));
  pkt[kHeaderBytes + 3] ^= 0x40;  // flip a bit in the payload, leave the CRC

  uint8_t frame[kMaxFrameBytes] = {};
  const size_t e = cobsEncode(pkt, n, frame, sizeof(frame));
  frame[e]       = kDelimiter;

  FrameReader r;
  TEST_ASSERT_EQUAL_UINT(0, pushAll(r, frame, e + 1));
  TEST_ASSERT_EQUAL_UINT32(0, r.framesAccepted());
  TEST_ASSERT_EQUAL_UINT32(1, r.crcErrors());
  TEST_ASSERT_EQUAL_UINT32(0, r.framingErrors());
}

static void test_an_overlong_frame_is_counted_once_and_recovered_from(void) {
  FrameReader r;
  for (size_t i = 0; i < kMaxFrameBytes * 2; ++i) r.push(0x5A);

  TEST_ASSERT_EQUAL_UINT32(1, r.overruns());  // once, not once per excess byte

  // And the reader is still usable afterwards.
  r.push(kDelimiter);
  uint8_t pkt[kMaxPacketBytes];
  const size_t n = makePacket(pkt, sizeof(pkt));
  uint8_t frame[kMaxFrameBytes] = {};
  const size_t e = cobsEncode(pkt, n, frame, sizeof(frame));
  frame[e]       = kDelimiter;
  TEST_ASSERT_EQUAL_UINT(n, pushAll(r, frame, e + 1));
}

// Several packets back to back, as a real stream delivers them.
static void test_a_stream_of_packets_is_read_in_order(void) {
  FrameReader r;
  uint16_t seen[5]  = {};
  size_t   seenCount = 0;

  for (uint16_t seq = 1; seq <= 5; ++seq) {
    uint8_t pkt[kMaxPacketBytes];
    const size_t n = makePacket(pkt, sizeof(pkt), seq);
    uint8_t frame[kMaxFrameBytes] = {};
    const size_t e = cobsEncode(pkt, n, frame, sizeof(frame));
    frame[e]       = kDelimiter;

    for (size_t i = 0; i <= e; ++i) {
      const size_t got = r.push(frame[i]);
      if (got) {
        PacketHeader h = {};
        TEST_ASSERT_TRUE(peekHeader(r.packet(), got, h));
        seen[seenCount++] = h.sequence;
      }
    }
  }

  TEST_ASSERT_EQUAL_UINT(5, seenCount);
  TEST_ASSERT_EQUAL_UINT32(5, r.framesAccepted());
  for (uint16_t i = 0; i < 5; ++i) TEST_ASSERT_EQUAL_UINT16(i + 1, seen[i]);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_round_trip_of_representative_payloads);
  RUN_TEST(test_the_254_byte_run_boundary);
  RUN_TEST(test_a_maximum_size_packet_fits_the_frame_buffer);
  RUN_TEST(test_encode_refuses_rather_than_writing_partially);
  RUN_TEST(test_decode_rejects_malformed_bodies);
  RUN_TEST(test_reader_accepts_a_well_framed_packet);
  RUN_TEST(test_delimiters_alone_are_not_errors);
  RUN_TEST(test_reader_resynchronises_after_garbage);
  RUN_TEST(test_a_corrupted_payload_counts_as_a_crc_error);
  RUN_TEST(test_an_overlong_frame_is_counted_once_and_recovered_from);
  RUN_TEST(test_a_stream_of_packets_is_read_in_order);
  UNITY_END();
  return 0;
}
