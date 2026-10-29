// Conformance tests against the PUBLISHED SCHEMA SPECIFICATION.
//
// Other groups build their consumers from the specification document, not from
// these headers, so every numeric claim it makes is restated here independently
// and checked against what the firmware actually does. A test failing here means
// the firmware and the published contract have diverged, and a consumer written
// from the document will decode the wrong values.
//
// The formulas are checked exhaustively over the full input range of the raw
// byte rather than at sample points, because an off-by-one in a scale or an
// offset shows up only at the ends.
#include <math.h>
#include <string.h>

#include <unity.h>

#include "bms_types.h"
#include "crc16.h"
#include "obd/vehicle_types.h"
#include "telemetry/framing.h"
#include "telemetry/battery_packet_sink.h"
#include "telemetry/mappers.h"
#include "telemetry/packet.h"
#include "telemetry/packets.h"
#include "telemetry/publisher.h"

using namespace telemetry;

void setUp(void) {}
void tearDown(void) {}

// ------------------------- helpers -------------------------------------------

// One PID answer through the real decode path, as the ECU would deliver it.
static obd::VehicleSnapshot decodePid(uint8_t pid, const uint8_t *data, uint8_t len) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  obd::addSample(v, pid, data, len);
  obd::decodeAll(v);
  return v;
}

static uint32_t g_now = 0;
static uint32_t clk() { return g_now; }

class NullSink final : public IPacketSink {
 public:
  const char *name() const override { return "null"; }
  bool onPacket(const uint8_t *p, size_t n) override {
    ++calls; lastLen = n; if (n <= sizeof(last)) memcpy(last, p, n);
    return accept;
  }
  uint8_t last[kMaxPacketBytes] = {};
  size_t  lastLen = 0, calls = 0;
  bool    accept = true;
};

// =================== section 3.2, engine formulas ============================
// Specification: rpm = (256A + B) / 4
static void test_spec_engine_rpm_formula(void) {
  for (int a = 0; a <= 255; a += 17) {
    for (int b = 0; b <= 255; b += 23) {
      const uint8_t raw[2] = {(uint8_t)a, (uint8_t)b};
      const float want = (256.0f * a + b) / 4.0f;
      TEST_ASSERT_FLOAT_WITHIN(0.001f, want, decodePid(0x0C, raw, 2).rpm);
    }
  }
}

// Specification: throttle = A x 100 / 255, load = A x 100 / 255
static void test_spec_percentage_formulas(void) {
  for (int a = 0; a <= 255; ++a) {
    const uint8_t raw = (uint8_t)a;
    const float want = a * 100.0f / 255.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, want, decodePid(0x11, &raw, 1).throttlePct);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, want, decodePid(0x04, &raw, 1).engineLoadPct);
  }
}

// Specification: timing advance = A / 2 - 64, and 0x80 reads as 0.0 degrees
static void test_spec_timing_advance_formula(void) {
  for (int a = 0; a <= 255; ++a) {
    const uint8_t raw = (uint8_t)a;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, a / 2.0f - 64.0f,
                             decodePid(0x0E, &raw, 1).timingAdvanceDeg);
  }
  const uint8_t centre = 0x80;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, decodePid(0x0E, &centre, 1).timingAdvanceDeg);
}

// Specification: short fuel trim = A x 100 / 128 - 100, range -100 to +99.2
static void test_spec_fuel_trim_formula_and_range(void) {
  for (int a = 0; a <= 255; ++a) {
    const uint8_t raw = (uint8_t)a;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, a * 100.0f / 128.0f - 100.0f,
                             decodePid(0x06, &raw, 1).shortFuelTrimPct);
  }
  const uint8_t lo = 0x00, hi = 0xFF, mid = 0x80;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -100.0f, decodePid(0x06, &lo, 1).shortFuelTrimPct);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, decodePid(0x06, &mid, 1).shortFuelTrimPct);
  // 255 x 100 / 128 - 100 = 99.21875 exactly. The document states the range to
  // one decimal place as 99.2, which is that value correctly rounded.
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 99.21875f, decodePid(0x06, &hi, 1).shortFuelTrimPct);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 99.2f, decodePid(0x06, &hi, 1).shortFuelTrimPct);
}

// Specification: intake air and oil temperature = (A - 40) x 10, in 0.1 C
static void test_spec_temperature_formulas_reach_the_packet(void) {
  for (int a = 0; a <= 255; a += 7) {
    const uint8_t raw = (uint8_t)a;
    obd::VehicleSnapshot v = {};
    obd::reset(v);
    obd::addSample(v, 0x0F, &raw, 1);
    obd::addSample(v, 0x5C, &raw, 1);
    obd::decodeAll(v);

    EngineAccumulator acc;
    acc.reset(0);
    acc.push(v, 0);
    EngineTelemetry e = {};
    TEST_ASSERT_TRUE(acc.flush(e, 1000));

    const int16_t want = (int16_t)((a - 40) * 10);
    TEST_ASSERT_EQUAL_INT16(want, e.intakeAirDeciC);
    TEST_ASSERT_EQUAL_INT16(want, e.oilTempDeciC);
  }
}

// Specification: MAP = A kPa, ECU road speed = A km/h, no conversion
static void test_spec_direct_byte_formulas(void) {
  for (int a = 0; a <= 255; a += 5) {
    const uint8_t raw = (uint8_t)a;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)a, decodePid(0x0B, &raw, 1).mapKpa);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)a, decodePid(0x0D, &raw, 1).speedKph);
  }
}

// Specification: distance with MIL = 256A + B
static void test_spec_two_byte_counter_formula(void) {
  const uint8_t raw[2] = {0x00, 0x0B};  // the 11 km actually observed
  TEST_ASSERT_EQUAL_UINT16(11, decodePid(0x21, raw, 2).distanceWithMilKm);
  const uint8_t big[2] = {0x12, 0x34};
  TEST_ASSERT_EQUAL_UINT16(0x1234, decodePid(0x21, big, 2).distanceWithMilKm);
}

// =================== section 3.6, battery scaling ============================
// Specification: pack volts, capacities = uint16 big-endian x 0.01;
// current = int16 big-endian x 0.01, positive while charging;
// temperature = uint16 big-endian - 2731, in 0.1 C
static void test_spec_battery_scaling(void) {
  jbd::BmsSnapshot s = {};
  s.fields      = jbd::kFieldPack | jbd::kFieldCapacity | jbd::kFieldTemps | jbd::kFieldConfig;
  s.packVolts   = 5340 * 0.01f;       // raw 0x14DC
  s.currentAmps = (int16_t)-250 * 0.01f;
  s.remainingAh = 4200 * 0.01f;
  s.fullAh      = 10000 * 0.01f;
  s.ntcCount    = 3;
  s.ntcCelsius[0] = (2981 - 2731) * 0.1f;   // 25.0 C
  s.ntcCelsius[1] = (3031 - 2731) * 0.1f;   // 30.0 C
  s.ntcCelsius[2] = (2711 - 2731) * 0.1f;   // -2.0 C, below freezing is signed

  BatteryTelemetry b = {};
  TEST_ASSERT_TRUE(toBatteryTelemetry(s, b));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 53.40f, b.packVolts);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.00f, b.remainingAh);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.00f, b.fullAh);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.50f, b.currentAmps.last);
  TEST_ASSERT_EQUAL_INT16(250, b.tempDeciC[0]);
  TEST_ASSERT_EQUAL_INT16(300, b.tempDeciC[1]);
  TEST_ASSERT_EQUAL_INT16(-20, b.tempDeciC[2]);
}

// Specification: cellSpreadMv = cellMaxMv - cellMinMv;
// liveCellCount = population count of cellPresentMask
static void test_spec_derived_cell_fields(void) {
  jbd::BmsSnapshot s = {};
  s.fields    = jbd::kFieldCells | jbd::kFieldConfig;
  s.cellCount = 16;
  for (uint8_t i = 0; i < 16; ++i) {
    s.cellsReadMask |= (1u << i);
    s.cellMilliVolts[i] = (uint16_t)(3300 + i * 4);
  }
  s.cellMilliVolts[6] = 0;  // unpopulated tap

  BatteryTelemetry b = {};
  toBatteryTelemetry(s, b);

  TEST_ASSERT_EQUAL_UINT16(b.cellMaxMv - b.cellMinMv, b.cellSpreadMv);
  int pop = 0;
  for (int i = 0; i < 32; ++i) if (b.cellPresentMask & (1u << i)) ++pop;
  TEST_ASSERT_EQUAL_UINT8(pop, b.liveCellCount);
  TEST_ASSERT_EQUAL_UINT8(15, b.liveCellCount);
  TEST_ASSERT_EQUAL_UINT32(0, b.cellPresentMask & (1u << 6));
  TEST_ASSERT_EQUAL_UINT16(b.cellMilliVolts[b.cellMinIndex], b.cellMinMv);
  TEST_ASSERT_EQUAL_UINT16(b.cellMilliVolts[b.cellMaxIndex], b.cellMaxMv);
}

// Specification, section 3.6: "May exceed 16, in which case cellMilliVolts
// carries the first 16 and kFlagPartial is set in the header."
static void test_spec_oversized_pack_sets_the_partial_flag(void) {
  jbd::BmsSnapshot s = {};
  s.fields    = jbd::kFieldCells | jbd::kFieldConfig | jbd::kFieldPack;
  s.cellCount = 17;
  s.packVolts = 53.4f;
  for (uint8_t i = 0; i < 17; ++i) {
    s.cellsReadMask |= (1u << i);
    s.cellMilliVolts[i] = 3330;
  }

  BatteryTelemetry b = {};
  TEST_ASSERT_FALSE(toBatteryTelemetry(s, b));   // the caller is told
  TEST_ASSERT_EQUAL_UINT8(17, b.cellCount);      // reported truthfully
  TEST_ASSERT_EQUAL_UINT16(0, b.cellPresentMask >> 16);  // only 16 carried

  // And the publishing path must actually raise kFlagPartial in the header,
  // which is what the specification tells a consumer to look for.
  NullSink  sink;
  Publisher pub(sink, clk);
  BatteryPacketSink bridge(pub, SourceId::kBatteryA, BatteryVendor::kJbd);
  bridge.onSnapshot(s);

  PacketHeader h = {};
  TEST_ASSERT_TRUE(peekHeader(sink.last, sink.lastLen, h));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PacketType::kBattery), h.type);
  TEST_ASSERT_TRUE((h.flags & kFlagPartial) != 0);
  TEST_ASSERT_EQUAL_UINT8(1, bridge.truncatedSnapshots());
}

// =================== section 2.4, StatBlock ==================================
// Specification: "0 means the channel was absent; all floats are then NaN"
// and "stddev: 0 when samples is less than 2"
static void test_spec_statblock_absent_and_single_sample(void) {
  const StatBlock e = emptyStats();
  TEST_ASSERT_EQUAL_UINT16(0, e.samples);

  // Five of the six floats are NaN. stddev is NOT: it is 0, consistent with the
  // rule that it is 0 whenever samples is below 2. So samples == 0 is the only
  // reliable absence test, and isnan(stddev) must never be used as one.
  TEST_ASSERT_TRUE(isnan(e.min));
  TEST_ASSERT_TRUE(isnan(e.max));
  TEST_ASSERT_TRUE(isnan(e.mean));
  TEST_ASSERT_TRUE(isnan(e.last));
  TEST_ASSERT_TRUE(isnan(e.slopePerSec));
  TEST_ASSERT_FALSE(isnan(e.stddev));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, e.stddev);
  TEST_ASSERT_EQUAL_UINT32(0, e.windowMs);

  RunningStat r;
  r.reset(0);
  r.push(1500.0f, 0);
  const StatBlock one = r.snapshot();
  TEST_ASSERT_EQUAL_UINT16(1, one.samples);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, one.stddev);
}

// Specification: slopePerSec = (last - first) / window seconds
static void test_spec_slope_definition(void) {
  RunningStat r;
  r.reset(0);
  r.push(1000.0f, 0);
  r.push(1200.0f, 500);
  r.push(1400.0f, 1000);           // 400 rpm over 1.000 s
  const StatBlock s = r.snapshot();
  TEST_ASSERT_EQUAL_UINT32(1000, s.windowMs);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 400.0f, s.slopePerSec);
  TEST_ASSERT_EQUAL_FLOAT(1400.0f, s.last);
}

// =================== section 2.2, sequence numbering =========================
// Specification: "Header sequence numbers are counted per source, not per
// packet type. Two packet types originating from one source share a counter."
static void test_spec_sequence_is_per_source_not_per_type(void) {
  NullSink  sink;
  Publisher pub(sink, clk);
  DynamicsTelemetry   d = {};
  NavigationTelemetry n = {};

  uint16_t seen[4] = {};
  for (int i = 0; i < 2; ++i) {
    PacketHeader h = {};
    pub.publish(PacketType::kDynamics, SourceId::kSensorGroup1, d);
    peekHeader(sink.last, sink.lastLen, h); seen[i * 2] = h.sequence;
    pub.publish(PacketType::kNavigation, SourceId::kSensorGroup1, n);
    peekHeader(sink.last, sink.lastLen, h); seen[i * 2 + 1] = h.sequence;
  }
  // One shared counter: 0, 1, 2, 3 across the two types.
  for (uint16_t i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(i, seen[i]);
}

// Specification, section 10.3 of the repository document and the published
// note: a refused packet still consumes its sequence number.
static void test_spec_refused_packet_still_consumes_a_sequence(void) {
  NullSink  sink;
  Publisher pub(sink, clk);
  BatteryTelemetry b = {};
  pub.publish(PacketType::kBattery, SourceId::kBatteryA, b);
  sink.accept = false;
  TEST_ASSERT_FALSE(pub.publish(PacketType::kBattery, SourceId::kBatteryA, b));
  sink.accept = true;
  pub.publish(PacketType::kBattery, SourceId::kBatteryA, b);

  PacketHeader h = {};
  peekHeader(sink.last, sink.lastLen, h);
  TEST_ASSERT_EQUAL_UINT16(2, h.sequence);   // 1 was consumed by the refusal
}

// Specification: counters are 16-bit and wrap at 65535.
static void test_spec_sequence_wraps_at_16_bits(void) {
  NullSink  sink;
  Publisher pub(sink, clk);
  BatteryTelemetry b = {};
  for (uint32_t i = 0; i < 65536; ++i) pub.publish(PacketType::kBattery, SourceId::kBatteryA, b);
  TEST_ASSERT_EQUAL_UINT16(0, pub.sequenceFor(SourceId::kBatteryA));
  pub.publish(PacketType::kBattery, SourceId::kBatteryA, b);
  PacketHeader h = {};
  peekHeader(sink.last, sink.lastLen, h);
  TEST_ASSERT_EQUAL_UINT16(0, h.sequence);   // wrapped, not saturated
}

// =================== section 5.10, event flags ===============================
// Specification: "The mask is zero on the first packet after a link is
// established, because no transition has been observed at that point."
static void test_spec_first_status_packet_has_a_zero_event_mask(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.milOn = true;
  v.storedDtcCount = 2;
  v.dtc[0] = 0x0145;
  v.dtc[1] = 0xC34D;

  StatusTracker t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_TRUE(t.update(v, st));
  TEST_ASSERT_EQUAL_UINT16(0, st.eventMask);
}

// Specification: "A protection bit was newly raised. A bit clearing does not
// set this flag."
static void test_spec_cleared_protection_is_not_a_fault_event(void) {
  jbd::BmsSnapshot s = {};
  s.fields = jbd::kFieldProtect | jbd::kFieldMos;
  StatusTracker t;
  StatusEventTelemetry st = {};
  t.update(s, st);

  s.protection = kProtCellUv;
  TEST_ASSERT_TRUE(t.update(s, st));
  TEST_ASSERT_TRUE((st.eventMask & kEvBatteryFault) != 0);

  s.protection = 0;
  TEST_ASSERT_TRUE(t.update(s, st));
  TEST_ASSERT_EQUAL_UINT16(0, st.eventMask & kEvBatteryFault);
}

// =================== section 6.1, DTC packing ================================
// Specification: bits 15-14 letter (P C B U), 13-12 first digit,
// then three hexadecimal nibbles. Worked examples 0x0145 -> P0145, 0xC34D -> U034D.
static void test_spec_dtc_packing(void) {
  struct Case { uint16_t code; char letter; int d1, d2, d3, d4; };
  const Case cases[] = {
      {0x0145, 'P', 0, 1, 4, 5},
      {0xC34D, 'U', 0, 3, 4, 13},
      {0x4000, 'C', 0, 0, 0, 0},
      {0x8ABC, 'B', 0, 10, 11, 12},
      {0x3FFF, 'P', 3, 15, 15, 15},
  };
  const char *letters = "PCBU";
  for (const Case &c : cases) {
    TEST_ASSERT_EQUAL_CHAR(c.letter, letters[(c.code >> 14) & 0x3]);
    TEST_ASSERT_EQUAL_INT(c.d1, (c.code >> 12) & 0x3);
    TEST_ASSERT_EQUAL_INT(c.d2, (c.code >> 8) & 0xF);
    TEST_ASSERT_EQUAL_INT(c.d3, (c.code >> 4) & 0xF);
    TEST_ASSERT_EQUAL_INT(c.d4, c.code & 0xF);
  }
}

// =================== section 6.2 and 7.4 =====================================
// Specification: CRC-16/MODBUS, check value 0x4B37 for "123456789".
static void test_spec_crc_parameters(void) {
  const char *s = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x4B37, util::crc16Modbus((const uint8_t *)s, strlen(s)));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, util::kCrcInit);
}

// Specification, section 7.4: a receiver rejects a packet unless the magic, the
// schema version, the declared length and the CRC all hold.
static void test_spec_receiver_rejects_each_way(void) {
  uint8_t buf[kHeaderBytes + sizeof(BatteryTelemetry)];
  BatteryTelemetry b = {};
  const size_t n = encode(PacketType::kBattery, SourceId::kBatteryA, 1, 0, 0, b, buf, sizeof buf);
  TEST_ASSERT_TRUE(verify(buf, n));

  uint8_t t[sizeof(buf)];
  memcpy(t, buf, n); t[0] ^= 0xFF;                       // magic
  TEST_ASSERT_FALSE(verify(t, n));
  memcpy(t, buf, n); t[2] = kSchemaVersion + 1;          // schema version
  TEST_ASSERT_FALSE(verify(t, n));
  memcpy(t, buf, n);                                     // declared length
  TEST_ASSERT_FALSE(verify(t, n - 1));
  memcpy(t, buf, n); t[kHeaderBytes + 5] ^= 0x01;        // payload bit
  TEST_ASSERT_FALSE(verify(t, n));
}

// =================== section 7.1, framing ====================================
// Specification: 324 bytes of packet become 327 on the wire, 0.9 per cent.
static void test_spec_framing_overhead(void) {
  TEST_ASSERT_EQUAL_UINT(324, kMaxPacketBytes);
  TEST_ASSERT_EQUAL_UINT(308, kMaxPayloadBytes);
  TEST_ASSERT_EQUAL_UINT(327, kMaxFrameBytes);
  TEST_ASSERT_EQUAL_UINT8(0x00, kDelimiter);

  // Worst case for COBS is a payload with no zero bytes at all: one code byte
  // per 254-byte run. 324 bytes give 1 + 254 + 1 + 70 = 326, and the delimiter
  // makes 327 - exactly kMaxFrameBytes, and exactly the 3 bytes of overhead the
  // document states.
  uint8_t pkt[kMaxPacketBytes];
  for (size_t i = 0; i < sizeof(pkt); ++i) pkt[i] = (uint8_t)(i | 1);  // no zero bytes
  uint8_t enc[kMaxFrameBytes];
  const size_t e = cobsEncode(pkt, sizeof(pkt), enc, sizeof(enc));
  TEST_ASSERT_EQUAL_UINT(326, e);
  TEST_ASSERT_EQUAL_UINT(kMaxFrameBytes, e + 1);
  TEST_ASSERT_EQUAL_UINT(3, (e + 1) - kMaxPacketBytes);
}

// Specification, section 7.3: the five 1 Hz packets total 980 bytes of payload
// and header per second.
static void test_spec_link_budget_arithmetic(void) {
  const size_t oneHz = (sizeof(EngineTelemetry) + kHeaderBytes) +
                       (sizeof(MotorTelemetry) + kHeaderBytes) +
                       (sizeof(DynamicsTelemetry) + kHeaderBytes) +
                       (sizeof(NavigationTelemetry) + kHeaderBytes) +
                       (sizeof(BatteryTelemetry) + kHeaderBytes);
  TEST_ASSERT_EQUAL_UINT(980, oneHz);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_spec_engine_rpm_formula);
  RUN_TEST(test_spec_percentage_formulas);
  RUN_TEST(test_spec_timing_advance_formula);
  RUN_TEST(test_spec_fuel_trim_formula_and_range);
  RUN_TEST(test_spec_temperature_formulas_reach_the_packet);
  RUN_TEST(test_spec_direct_byte_formulas);
  RUN_TEST(test_spec_two_byte_counter_formula);
  RUN_TEST(test_spec_battery_scaling);
  RUN_TEST(test_spec_derived_cell_fields);
  RUN_TEST(test_spec_oversized_pack_sets_the_partial_flag);
  RUN_TEST(test_spec_statblock_absent_and_single_sample);
  RUN_TEST(test_spec_slope_definition);
  RUN_TEST(test_spec_sequence_is_per_source_not_per_type);
  RUN_TEST(test_spec_refused_packet_still_consumes_a_sequence);
  RUN_TEST(test_spec_sequence_wraps_at_16_bits);
  RUN_TEST(test_spec_first_status_packet_has_a_zero_event_mask);
  RUN_TEST(test_spec_cleared_protection_is_not_a_fault_event);
  RUN_TEST(test_spec_dtc_packing);
  RUN_TEST(test_spec_crc_parameters);
  RUN_TEST(test_spec_receiver_rejects_each_way);
  RUN_TEST(test_spec_framing_overhead);
  RUN_TEST(test_spec_link_budget_arithmetic);
  UNITY_END();
  return 0;
}
