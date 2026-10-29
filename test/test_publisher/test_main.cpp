// Host tests for the publish path: the API an application calls, the queue that
// hands packets between tasks, and the sink that frames them for a UART.
//
// The whole path is exercised here with a byte writer that appends to an array,
// which is the reason IByteWriter exists at all.
#include <string.h>

#include <unity.h>

#include "telemetry/framing.h"
#include "telemetry/packet.h"
#include "telemetry/packet_queue.h"
#include "telemetry/packets.h"
#include "telemetry/publisher.h"

using namespace telemetry;

void setUp(void) {}
void tearDown(void) {}

// ------------------------------- test doubles -------------------------------

// A writer that records everything, and can be told to go short or fail.
class CaptureWriter final : public IByteWriter {
 public:
  size_t write(const uint8_t *data, size_t length) override {
    ++calls;
    const size_t take = (limit == 0) ? length : (length < limit ? length : limit);
    for (size_t i = 0; i < take && used < sizeof(bytes); ++i) bytes[used++] = data[i];
    return take;
  }
  uint8_t bytes[4096] = {};
  size_t  used        = 0;
  size_t  calls       = 0;
  size_t  limit       = 0;  // 0 = accept everything
};

// A sink that counts and can refuse.
class CountingSink final : public IPacketSink {
 public:
  const char *name() const override { return "counting"; }
  bool onPacket(const uint8_t *packet, size_t length) override {
    ++calls;
    lastLength = length;
    if (length <= sizeof(last)) memcpy(last, packet, length);
    return accept;
  }
  uint8_t last[kMaxPacketBytes] = {};
  size_t  lastLength            = 0;
  size_t  calls                 = 0;
  bool    accept                = true;
};

static uint32_t g_now = 0;
static uint32_t testClock() { return g_now; }

static BatteryTelemetry someBattery(float volts = 53.4f) {
  BatteryTelemetry b = {};
  b.packVolts        = volts;
  b.cellCount        = 16;
  b.liveCellCount    = 15;
  b.fields           = kBatPack | kBatCells;
  return b;
}

// --------------------------------- publisher --------------------------------

static void test_publish_builds_a_verifiable_packet(void) {
  CountingSink sink;
  Publisher    pub(sink, testClock);
  g_now = 4242;

  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  TEST_ASSERT_EQUAL_UINT(1, sink.calls);
  TEST_ASSERT_EQUAL_UINT(kHeaderBytes + sizeof(BatteryTelemetry), sink.lastLength);
  TEST_ASSERT_TRUE(verify(sink.last, sink.lastLength));

  PacketHeader h = {};
  TEST_ASSERT_TRUE(peekHeader(sink.last, sink.lastLength, h));
  TEST_ASSERT_EQUAL_UINT16(kMagic, h.magic);
  TEST_ASSERT_EQUAL_UINT8(kSchemaVersion, h.schemaVersion);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PacketType::kBattery), h.type);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SourceId::kBatteryA), h.source);
  TEST_ASSERT_EQUAL_UINT32(4242, h.uptimeMs);

  // The payload arrives byte-identical.
  const BatteryTelemetry *got =
      static_cast<const BatteryTelemetry *>(payloadOf(sink.last, sink.lastLength));
  TEST_ASSERT_NOT_NULL(got);
  TEST_ASSERT_EQUAL_FLOAT(53.4f, got->packVolts);
  TEST_ASSERT_EQUAL_UINT8(16, got->cellCount);
}

// The header documents sequence numbers as per-source. If they were global, a
// consumer watching one source would see gaps caused purely by another source
// publishing, and could not tell real loss from interleaving.
static void test_sequence_numbers_are_independent_per_source(void) {
  CountingSink sink;
  Publisher    pub(sink, testClock);

  uint16_t ecuSeq[3] = {};
  uint16_t batSeq[3] = {};

  for (int i = 0; i < 3; ++i) {
    EngineTelemetry e = {};
    pub.publish(PacketType::kEngine, SourceId::kEcu, e);
    PacketHeader h = {};
    peekHeader(sink.last, sink.lastLength, h);
    ecuSeq[i] = h.sequence;

    pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery());
    peekHeader(sink.last, sink.lastLength, h);
    batSeq[i] = h.sequence;
  }

  for (int i = 0; i < 3; ++i) {
    TEST_ASSERT_EQUAL_UINT16(i, ecuSeq[i]);
    TEST_ASSERT_EQUAL_UINT16(i, batSeq[i]);
  }
  TEST_ASSERT_EQUAL_UINT16(3, pub.sequenceFor(SourceId::kEcu));
  TEST_ASSERT_EQUAL_UINT16(3, pub.sequenceFor(SourceId::kBatteryA));
  TEST_ASSERT_EQUAL_UINT16(0, pub.sequenceFor(SourceId::kMotorDriver));
}

static void test_a_refusing_sink_is_counted_not_hidden(void) {
  CountingSink sink;
  Publisher    pub(sink, testClock);

  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  sink.accept = false;
  TEST_ASSERT_FALSE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));

  TEST_ASSERT_EQUAL_UINT32(1, pub.publishedCount());
  TEST_ASSERT_EQUAL_UINT32(1, pub.refusedCount());

  // A refused packet still consumed its sequence number, so the gap is visible
  // downstream rather than the stream silently renumbering itself.
  TEST_ASSERT_EQUAL_UINT16(2, pub.sequenceFor(SourceId::kBatteryA));
}

static void test_an_oversized_payload_is_rejected(void) {
  CountingSink sink;
  Publisher    pub(sink, testClock);

  uint8_t big[kMaxPayloadBytes + 64] = {};
  TEST_ASSERT_FALSE(pub.publishRaw(PacketType::kDynamics, SourceId::kSensorGroup1, big,
                                   sizeof(big), 0));
  TEST_ASSERT_EQUAL_UINT32(1, pub.encodeErrors());
  TEST_ASSERT_EQUAL_UINT(0, sink.calls);
}

static void test_a_null_clock_does_not_crash(void) {
  CountingSink sink;
  Publisher    pub(sink, nullptr);
  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  PacketHeader h = {};
  peekHeader(sink.last, sink.lastLength, h);
  TEST_ASSERT_EQUAL_UINT32(0, h.uptimeMs);
}

// ----------------------------------- queue ----------------------------------

static void test_queue_preserves_order(void) {
  PacketQueue<4> q;
  CountingSink   discard;
  QueueSink<4>   sink(q);
  Publisher      pub(sink, testClock);

  for (int i = 0; i < 3; ++i) pub.publish(PacketType::kBattery, SourceId::kBatteryA,
                                          someBattery(static_cast<float>(50 + i)));
  TEST_ASSERT_EQUAL_UINT(3, q.count());

  for (int i = 0; i < 3; ++i) {
    uint8_t out[kMaxPacketBytes] = {};
    const size_t n = q.pop(out, sizeof(out));
    TEST_ASSERT_GREATER_THAN_UINT(0, n);
    TEST_ASSERT_TRUE(verify(out, n));
    PacketHeader h = {};
    peekHeader(out, n, h);
    TEST_ASSERT_EQUAL_UINT16(i, h.sequence);  // oldest first
  }
  TEST_ASSERT_TRUE(q.empty());
  TEST_ASSERT_EQUAL_UINT(0, q.pop(nullptr, 0));
  (void)discard;
}

// Newest-wins: the OLDEST is discarded, and the discard is counted so a
// consumer can tell it fell behind rather than quietly missing packets.
static void test_queue_overflow_drops_the_oldest_and_counts_it(void) {
  PacketQueue<4> q;
  QueueSink<4>   sink(q);
  Publisher      pub(sink, testClock);

  for (int i = 0; i < 6; ++i) pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery());

  TEST_ASSERT_EQUAL_UINT(4, q.count());
  TEST_ASSERT_EQUAL_UINT32(6, q.pushedCount());
  TEST_ASSERT_EQUAL_UINT32(2, q.droppedCount());

  // What survived is the four NEWEST, i.e. sequences 2..5.
  for (uint16_t expect = 2; expect <= 5; ++expect) {
    uint8_t out[kMaxPacketBytes] = {};
    const size_t n = q.pop(out, sizeof(out));
    PacketHeader h = {};
    peekHeader(out, n, h);
    TEST_ASSERT_EQUAL_UINT16(expect, h.sequence);
  }
}

static void test_queue_rejects_what_can_never_fit(void) {
  PacketQueue<2> q;
  uint8_t        big[kMaxPacketBytes + 1] = {};

  TEST_ASSERT_FALSE(q.push(big, sizeof(big)));
  TEST_ASSERT_FALSE(q.push(nullptr, 10));
  TEST_ASSERT_FALSE(q.push(big, 0));
  TEST_ASSERT_EQUAL_UINT32(3, q.rejectedCount());
  TEST_ASSERT_EQUAL_UINT32(0, q.pushedCount());
  TEST_ASSERT_TRUE(q.empty());
}

// An undersized destination must leave the packet queued, not consume it.
static void test_pop_into_a_small_buffer_loses_nothing(void) {
  PacketQueue<2> q;
  QueueSink<2>   sink(q);
  Publisher      pub(sink, testClock);
  pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery());

  uint8_t tiny[8] = {};
  TEST_ASSERT_EQUAL_UINT(0, q.pop(tiny, sizeof(tiny)));
  TEST_ASSERT_EQUAL_UINT(1, q.count());

  uint8_t out[kMaxPacketBytes] = {};
  TEST_ASSERT_GREATER_THAN_UINT(0, q.pop(out, sizeof(out)));
}

// ------------------------- the whole path, end to end -----------------------

// Publish -> COBS -> bytes -> FrameReader -> the same packet. This is the test
// that says the firmware and a host decoder agree.
static void test_publish_through_cobs_and_back(void) {
  CaptureWriter  writer;
  CobsFrameSink  cobs(writer);
  Publisher      pub(cobs, testClock);
  g_now = 777;

  const BatteryTelemetry sent = someBattery(48.25f);
  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryB, sent));

  // One write call, body and delimiter together.
  TEST_ASSERT_EQUAL_UINT(1, writer.calls);
  TEST_ASSERT_EQUAL_UINT8(kDelimiter, writer.bytes[writer.used - 1]);
  TEST_ASSERT_EQUAL_UINT32(1, cobs.framesWritten());

  FrameReader reader;
  size_t      got = 0;
  for (size_t i = 0; i < writer.used; ++i) {
    const size_t n = reader.push(writer.bytes[i]);
    if (n) got = n;
  }

  TEST_ASSERT_EQUAL_UINT(kHeaderBytes + sizeof(BatteryTelemetry), got);
  TEST_ASSERT_EQUAL_UINT32(1, reader.framesAccepted());
  TEST_ASSERT_EQUAL_UINT32(0, reader.crcErrors());

  PacketHeader h = {};
  TEST_ASSERT_TRUE(peekHeader(reader.packet(), got, h));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SourceId::kBatteryB), h.source);
  TEST_ASSERT_EQUAL_UINT32(777, h.uptimeMs);

  const BatteryTelemetry *back =
      static_cast<const BatteryTelemetry *>(payloadOf(reader.packet(), got));
  TEST_ASSERT_NOT_NULL(back);
  TEST_ASSERT_EQUAL_MEMORY(&sent, back, sizeof(sent));
}

// A transport that cannot take the whole frame must lose the packet outright. A
// partial frame would cost the receiver a resynchronisation and corrupt the
// packet that follows.
static void test_a_short_write_reports_failure(void) {
  CaptureWriter writer;
  writer.limit = 10;  // accept only 10 bytes of each frame
  CobsFrameSink cobs(writer);
  Publisher     pub(cobs, testClock);

  TEST_ASSERT_FALSE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  TEST_ASSERT_EQUAL_UINT32(1, cobs.shortWrites());
  TEST_ASSERT_EQUAL_UINT32(0, cobs.framesWritten());
  TEST_ASSERT_EQUAL_UINT32(1, pub.refusedCount());
}

// All seven types through the full path, so no payload size breaks framing.
static void test_every_packet_type_survives_the_whole_path(void) {
  CaptureWriter writer;
  CobsFrameSink cobs(writer);
  Publisher     pub(cobs, testClock);

  IdentityTelemetry    id  = {};
  EngineTelemetry      en  = {};
  MotorTelemetry       mo  = {};
  DynamicsTelemetry    dy  = {};
  NavigationTelemetry  nav = {};
  BatteryTelemetry     bat = {};
  StatusEventTelemetry st  = {};

  TEST_ASSERT_TRUE(pub.publish(PacketType::kIdentity, SourceId::kEcu, id));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kEngine, SourceId::kEcu, en));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kMotor, SourceId::kMotorDriver, mo));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kDynamics, SourceId::kSensorGroup1, dy));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kNavigation, SourceId::kSensorGroup1, nav));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, bat));
  TEST_ASSERT_TRUE(pub.publish(PacketType::kStatusEvent, SourceId::kFusion, st));

  FrameReader reader;
  size_t      accepted = 0;
  for (size_t i = 0; i < writer.used; ++i) {
    if (reader.push(writer.bytes[i])) ++accepted;
  }
  TEST_ASSERT_EQUAL_UINT(7, accepted);
  TEST_ASSERT_EQUAL_UINT32(0, reader.framingErrors() + reader.crcErrors());
}

// ---------------------------------- fan-out ---------------------------------

static void test_fan_out_reaches_every_sink_and_reports_the_worst(void) {
  CountingSink a, b;
  FanOutSink<4> fan;
  TEST_ASSERT_TRUE(fan.add(&a));
  TEST_ASSERT_TRUE(fan.add(&b));

  Publisher pub(fan, testClock);
  TEST_ASSERT_TRUE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  TEST_ASSERT_EQUAL_UINT(1, a.calls);
  TEST_ASSERT_EQUAL_UINT(1, b.calls);

  // One sink failing must not be masked by the other succeeding, or a dead
  // transport hides behind a working debug sink.
  b.accept = false;
  TEST_ASSERT_FALSE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, someBattery()));
  TEST_ASSERT_EQUAL_UINT(2, a.calls);

  // An empty fan-out reports failure rather than pretending to have delivered.
  FanOutSink<2> empty;
  TEST_ASSERT_FALSE(empty.onPacket(a.last, a.lastLength));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_publish_builds_a_verifiable_packet);
  RUN_TEST(test_sequence_numbers_are_independent_per_source);
  RUN_TEST(test_a_refusing_sink_is_counted_not_hidden);
  RUN_TEST(test_an_oversized_payload_is_rejected);
  RUN_TEST(test_a_null_clock_does_not_crash);
  RUN_TEST(test_queue_preserves_order);
  RUN_TEST(test_queue_overflow_drops_the_oldest_and_counts_it);
  RUN_TEST(test_queue_rejects_what_can_never_fit);
  RUN_TEST(test_pop_into_a_small_buffer_loses_nothing);
  RUN_TEST(test_publish_through_cobs_and_back);
  RUN_TEST(test_a_short_write_reports_failure);
  RUN_TEST(test_every_packet_type_survives_the_whole_path);
  RUN_TEST(test_fan_out_reaches_every_sink_and_reports_the_worst);
  UNITY_END();
  return 0;
}
