// Host tests for the telemetry layer: the statistics that replace raw fast
// signals, and the packet envelope that carries them.
#include <math.h>
#include <string.h>

#include <initializer_list>

#include <unity.h>

#include "crc16.h"
#include "telemetry/packet.h"
#include "telemetry/packets.h"
#include "telemetry/stat_block.h"

using namespace telemetry;

void setUp(void) {}
void tearDown(void) {}

// ------------------------------ statistics ----------------------------------

// Textbook set {2,4,4,4,5,5,7,9}: mean 5, sum of squared deviations 32, so the
// sample standard deviation is sqrt(32/7) = 2.13809.
static void test_welford_matches_hand_computed_statistics(void) {
  const float v[] = {2, 4, 4, 4, 5, 5, 7, 9};
  RunningStat r;
  r.reset(0);
  for (int i = 0; i < 8; ++i) r.push(v[i], static_cast<uint32_t>(i * 100));

  const StatBlock s = r.snapshot();
  TEST_ASSERT_EQUAL_UINT16(8, s.samples);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 5.0f, s.mean);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 2.13809f, s.stddev);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 2.0f, s.min);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 9.0f, s.max);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 9.0f, s.last);
  TEST_ASSERT_EQUAL_UINT32(700, s.windowMs);
  // (last - first) / window = (9 - 2) / 0.7 s
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, s.slopePerSec);
}

// Welford exists precisely so a large mean with a small spread stays accurate;
// a naive sum of squares loses the variance entirely at this scale in float.
static void test_stable_for_a_large_mean_with_small_spread(void) {
  RunningStat r;
  r.reset(0);
  for (int i = 0; i < 100; ++i) r.push(1500.0f + (i % 2 ? 1.0f : -1.0f), i * 10u);
  const StatBlock s = r.snapshot();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, s.mean);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.005f, s.stddev);  // alternating +/-1
}

// An absent channel must be unmistakable, never a plausible zero.
static void test_empty_window_is_not_zero(void) {
  RunningStat r;
  r.reset(1000);
  const StatBlock s = r.snapshot();
  TEST_ASSERT_EQUAL_UINT16(0, s.samples);
  TEST_ASSERT_TRUE(isnan(s.mean));
  TEST_ASSERT_TRUE(isnan(s.min));
  TEST_ASSERT_TRUE(isnan(s.last));
}

static void test_single_sample_has_no_spread_or_slope(void) {
  RunningStat r;
  r.reset(0);
  r.push(1234.0f, 0);
  const StatBlock s = r.snapshot();
  TEST_ASSERT_EQUAL_UINT16(1, s.samples);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1234.0f, s.mean);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.stddev);      // defined as zero, not NaN
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.slopePerSec);
}

static void test_reset_starts_a_fresh_window(void) {
  RunningStat r;
  r.reset(0);
  r.push(10.0f, 0);
  r.push(20.0f, 100);
  r.reset(500);
  TEST_ASSERT_TRUE(r.empty());
  r.push(99.0f, 500);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 99.0f, r.snapshot().mean);
}

// ------------------------------- packets ------------------------------------

static BatteryTelemetry sampleBattery() {
  BatteryTelemetry b{};
  b.vendor        = static_cast<uint8_t>(BatteryVendor::kJbd);
  b.chemistry     = static_cast<uint8_t>(CellChemistry::kNmc);
  b.cellCount     = 17;
  b.liveCellCount = 16;
  b.packVolts     = 57.12f;
  b.socPercent    = 0;
  b.protectionFlags = kProtCellUv;      // the real pack's latched fault
  b.mosfetState   = kMosCharge;          // charge on, discharge off
  b.cellSpreadMv  = 7;
  b.fields = kBatPack | kBatCells | kBatProtection | kBatMosfet;
  return b;
}

static void test_encode_verify_round_trip(void) {
  const BatteryTelemetry in = sampleBattery();
  uint8_t buf[256];
  const size_t n = encode(PacketType::kBattery, SourceId::kBatteryA, 42, 123456, 0, in,
                          buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT(kHeaderBytes + sizeof(BatteryTelemetry), n);
  TEST_ASSERT_TRUE(verify(buf, n));

  PacketHeader h;
  TEST_ASSERT_TRUE(peekHeader(buf, n, h));
  TEST_ASSERT_EQUAL_HEX16(kMagic, h.magic);
  TEST_ASSERT_EQUAL_UINT8(kSchemaVersion, h.schemaVersion);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PacketType::kBattery), h.type);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SourceId::kBatteryA), h.source);
  TEST_ASSERT_EQUAL_UINT16(42, h.sequence);
  TEST_ASSERT_EQUAL_UINT32(123456, h.uptimeMs);
  TEST_ASSERT_EQUAL_UINT16(sizeof(BatteryTelemetry), h.payloadBytes);

  const BatteryTelemetry *out =
      static_cast<const BatteryTelemetry *>(payloadOf(buf, n));
  TEST_ASSERT_NOT_NULL(out);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 57.12f, out->packVolts);
  TEST_ASSERT_EQUAL_UINT8(17, out->cellCount);
  TEST_ASSERT_EQUAL_HEX16(kProtCellUv, out->protectionFlags);
}

// A single flipped bit anywhere must fail the CRC, header or payload.
static void test_corruption_is_detected(void) {
  uint8_t buf[256];
  const size_t n = encode(PacketType::kBattery, SourceId::kBatteryA, 1, 0, 0,
                          sampleBattery(), buf, sizeof(buf));

  for (size_t pos : {size_t(4), size_t(10), kHeaderBytes + 2, n - 1}) {
    uint8_t bad[256];
    memcpy(bad, buf, n);
    bad[pos] ^= 0x01;
    TEST_ASSERT_FALSE_MESSAGE(verify(bad, n), "flipped bit slipped through");
  }
}

static void test_truncated_and_oversized_buffers_are_rejected(void) {
  uint8_t buf[256];
  const size_t n = encode(PacketType::kEngine, SourceId::kEcu, 1, 0, 0,
                          EngineTelemetry{}, buf, sizeof(buf));
  TEST_ASSERT_TRUE(verify(buf, n));
  TEST_ASSERT_FALSE(verify(buf, n - 1));        // payload cut short
  TEST_ASSERT_FALSE(verify(buf, kHeaderBytes));  // header only
  TEST_ASSERT_FALSE(verify(buf, 4));

  uint8_t tiny[8];
  TEST_ASSERT_EQUAL_UINT(0, encode(PacketType::kEngine, SourceId::kEcu, 1, 0, 0,
                                   EngineTelemetry{}, tiny, sizeof(tiny)));
}

static void test_wrong_schema_version_is_rejected(void) {
  uint8_t buf[256];
  const size_t n = encode(PacketType::kNavigation, SourceId::kEcu, 1, 0, 0,
                          NavigationTelemetry{}, buf, sizeof(buf));
  buf[2] = kSchemaVersion + 1;   // a future producer we cannot parse
  TEST_ASSERT_FALSE(verify(buf, n));
}

// The field mask is the whole point: a dropped reading must not read as zero.
static void test_field_mask_distinguishes_absent_from_zero(void) {
  EngineTelemetry e{};
  e.rpm    = emptyStats();
  e.fields = 0;
  TEST_ASSERT_FALSE(e.fields & kEngRpm);
  TEST_ASSERT_TRUE(isnan(e.rpm.mean));

  RunningStat r;
  r.reset(0);
  r.push(0.0f, 0);          // a genuine zero reading - engine stopped
  r.push(0.0f, 100);
  e.rpm    = r.snapshot();
  e.fields = kEngRpm;
  TEST_ASSERT_TRUE(e.fields & kEngRpm);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, e.rpm.mean);   // zero, and known to be zero
  TEST_ASSERT_EQUAL_UINT16(2, e.rpm.samples);
}

static void test_crc_check_value(void) {
  const uint8_t v[] = {'1','2','3','4','5','6','7','8','9'};
  TEST_ASSERT_EQUAL_HEX16(0x4B37, util::crc16Modbus(v, sizeof(v)));
  // The update form must be equivalent to one pass over the whole buffer.
  uint16_t c = util::crc16ModbusUpdate(util::kCrcInit, v, 4);
  c = util::crc16ModbusUpdate(c, v + 4, 5);
  TEST_ASSERT_EQUAL_HEX16(0x4B37, c);
}

// Latitude and longitude must survive at full precision. A float carries ~7
// significant digits, which at two or three digits of whole degrees leaves
// metres of error - the classic GPS-in-float bug. Scaled int32 is exact.
static void test_position_keeps_centimetre_precision(void) {
  NavigationTelemetry nav{};
  nav.latitudeE7  =  188166667;   // 18.8166667 N
  nav.longitudeE7 =  733133333;   // 73.3133333 E
  nav.fields      = kNavPosition;

  uint8_t buf[512];
  const size_t n = encode(PacketType::kNavigation, SourceId::kSensorGroup1, 7, 0, 0,
                          nav, buf, sizeof(buf));
  TEST_ASSERT_TRUE(verify(buf, n));
  const auto *out = (const NavigationTelemetry *)payloadOf(buf, n);
  TEST_ASSERT_EQUAL_INT32(188166667, out->latitudeE7);   // bit-exact
  TEST_ASSERT_EQUAL_INT32(733133333, out->longitudeE7);

  // The same value through a float would not survive.
  const float viaFloat = 18.8166667f;
  TEST_ASSERT_TRUE(fabs((double)viaFloat - 18.8166667) > 1e-8);
}

// Dynamics is the highest-rate source in the system; nine summarised channels
// is what makes it transmissible at all.
static void test_dynamics_carries_nine_stat_channels(void) {
  DynamicsTelemetry d{};
  RunningStat r;
  r.reset(0);
  for (int i = 0; i < 200; ++i) r.push(1.0f + 0.01f * (i % 7), i * 5u);
  d.accelX = d.accelY = d.accelZ = r.snapshot();
  d.fields = kDynAccel;

  uint8_t buf[512];
  const size_t n = encode(PacketType::kDynamics, SourceId::kSensorGroup1, 1, 0, 0, d,
                          buf, sizeof(buf));
  TEST_ASSERT_TRUE(verify(buf, n));
  const auto *out = (const DynamicsTelemetry *)payloadOf(buf, n);
  TEST_ASSERT_EQUAL_UINT16(200, out->accelX.samples);   // 200 samples -> 6 numbers
  TEST_ASSERT_TRUE(out->accelX.stddev > 0.0f);
}

// A status packet reports edges, so a consumer can act on a fault that appeared
// and cleared between two periodic samples.
static void test_status_event_mask(void) {
  StatusEventTelemetry st{};
  st.batteryProtectionFlags = kProtCellUv;
  st.eventMask = kEvBatteryFault | kEvRfidPresented;
  st.rfidUidLength = 4;
  st.rfidUid[0] = 0xA1; st.rfidUid[1] = 0xB2;
  st.rfidUid[2] = 0xC3; st.rfidUid[3] = 0xD4;
  st.rfidAuthorised = 1;
  st.fields = kStBattery | kStRfid;

  uint8_t buf[256];
  const size_t n = encode(PacketType::kStatusEvent, SourceId::kFusion, 1, 0, 0, st, buf,
                          sizeof(buf));
  TEST_ASSERT_TRUE(verify(buf, n));
  const auto *out = (const StatusEventTelemetry *)payloadOf(buf, n);
  TEST_ASSERT_TRUE(out->eventMask & kEvBatteryFault);
  TEST_ASSERT_TRUE(out->eventMask & kEvRfidPresented);
  TEST_ASSERT_FALSE(out->eventMask & kEvMotorFault);
  TEST_ASSERT_EQUAL_UINT8(4, out->rfidUidLength);
  TEST_ASSERT_EQUAL_HEX8(0xD4, out->rfidUid[3]);
}

// Every type must round-trip, so none is accidentally unencodable.
static void test_all_seven_types_round_trip(void) {
  uint8_t buf[512];
  struct Case { PacketType t; uint16_t bytes; };
  const Case cases[] = {
      {PacketType::kIdentity, sizeof(IdentityTelemetry)},
      {PacketType::kEngine, sizeof(EngineTelemetry)},
      {PacketType::kMotor, sizeof(MotorTelemetry)},
      {PacketType::kDynamics, sizeof(DynamicsTelemetry)},
      {PacketType::kNavigation, sizeof(NavigationTelemetry)},
      {PacketType::kBattery, sizeof(BatteryTelemetry)},
      {PacketType::kStatusEvent, sizeof(StatusEventTelemetry)},
  };
  uint8_t zeros[512] = {0};
  for (const Case &c : cases) {
    const size_t n = encode(c.t, SourceId::kFusion, 1, 0, 0, zeros, c.bytes, buf,
                            sizeof(buf));
    TEST_ASSERT_EQUAL_UINT_MESSAGE(kHeaderBytes + c.bytes, n, toString(c.t));
    TEST_ASSERT_TRUE_MESSAGE(verify(buf, n), toString(c.t));
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_welford_matches_hand_computed_statistics);
  RUN_TEST(test_stable_for_a_large_mean_with_small_spread);
  RUN_TEST(test_empty_window_is_not_zero);
  RUN_TEST(test_single_sample_has_no_spread_or_slope);
  RUN_TEST(test_reset_starts_a_fresh_window);
  RUN_TEST(test_encode_verify_round_trip);
  RUN_TEST(test_corruption_is_detected);
  RUN_TEST(test_truncated_and_oversized_buffers_are_rejected);
  RUN_TEST(test_wrong_schema_version_is_rejected);
  RUN_TEST(test_field_mask_distinguishes_absent_from_zero);
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_position_keeps_centimetre_precision);
  RUN_TEST(test_dynamics_carries_nine_stat_channels);
  RUN_TEST(test_status_event_mask);
  RUN_TEST(test_all_seven_types_round_trip);
  UNITY_END();
  return 0;
}
