// Host tests for the mapping layer. A mapping bug produces a perfectly
// well-formed packet that says the wrong thing, so it is invisible on the wire
// and can only be caught here.
#include <math.h>
#include <string.h>

#include <unity.h>

#include "bms_types.h"
#include "obd/vehicle_types.h"
#include "telemetry/mappers.h"
#include "telemetry/packets.h"

using namespace telemetry;

void setUp(void) {}
void tearDown(void) {}

// A plausible 16S-on-17-channels pack, as the bench BMS actually reports it:
// channel 10 reads exactly 0 mV because the tap is unpopulated.
static jbd::BmsSnapshot benchPack() {
  jbd::BmsSnapshot s = {};
  s.valid     = true;
  s.uptimeMs  = 10000;
  s.source    = jbd::Source::kUart;
  s.fields    = jbd::kFieldPack | jbd::kFieldCapacity | jbd::kFieldProtect |
                jbd::kFieldMos | jbd::kFieldConfig | jbd::kFieldTemps | jbd::kFieldCells;
  s.cellCount = 17;
  s.ntcCount  = 5;

  s.packVolts   = 53.4f;
  s.currentAmps = -2.5f;
  s.remainingAh = 42.0f;
  s.fullAh      = 100.0f;
  s.cycles      = 12;
  s.socPercent  = 42;

  s.protection     = jbd::kCellUv;
  s.balanceBits    = 0x5u;
  s.chargeMosOn    = true;
  s.dischargeMosOn = false;

  for (uint8_t i = 0; i < 17; ++i) {
    s.cellsReadMask |= (1u << i);
    s.cellMilliVolts[i] = static_cast<uint16_t>(3330 + i);
  }
  s.cellMilliVolts[9] = 0;   // channel 10, the dead tap
  s.cellMilliVolts[16] = 0;  // channel 17, configured but not wired

  for (uint8_t i = 0; i < 5; ++i) s.ntcCelsius[i] = 25.0f + i;

  jbd::computeDerived(s);
  return s;
}

// ------------------------------- battery ------------------------------------

static void test_battery_mapping_carries_the_pack(void) {
  const jbd::BmsSnapshot s = benchPack();
  BatteryTelemetry       b = {};

  // 17 configured channels do not fit the schema's 16, and the caller is told.
  TEST_ASSERT_FALSE(toBatteryTelemetry(s, b));

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryVendor::kJbd), b.vendor);
  TEST_ASSERT_EQUAL_FLOAT(53.4f, b.packVolts);
  TEST_ASSERT_EQUAL_FLOAT(42.0f, b.remainingAh);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, b.fullAh);
  TEST_ASSERT_EQUAL_UINT8(42, b.socPercent);
  TEST_ASSERT_EQUAL_UINT16(12, b.cycleCount);

  // cellCount is the truth about the BMS, even though only 16 fitted.
  TEST_ASSERT_EQUAL_UINT8(17, b.cellCount);

  // The current is a one-sample window, not a bare float.
  TEST_ASSERT_EQUAL_UINT16(1, b.currentAmps.samples);
  TEST_ASSERT_EQUAL_FLOAT(-2.5f, b.currentAmps.last);

  TEST_ASSERT_EQUAL_UINT16(jbd::kCellUv, b.protectionFlags);
  TEST_ASSERT_EQUAL_UINT8(kMosCharge, b.mosfetState);  // discharge is off
  TEST_ASSERT_EQUAL_UINT32(0x5u, b.balancingMask);

  const uint16_t want = kBatPack | kBatCurrent | kBatCapacity | kBatCells |
                        kBatTemps | kBatProtection | kBatMosfet | kBatBalance;
  TEST_ASSERT_EQUAL_UINT16(want, b.fields);
}

// The dead tap must be excluded from the spread, or the single most useful
// pack-health number reads as 3345 mV of imbalance on a healthy pack.
static void test_the_dead_channel_is_marked_absent_not_zero(void) {
  const jbd::BmsSnapshot s = benchPack();
  BatteryTelemetry       b = {};
  toBatteryTelemetry(s, b);

  TEST_ASSERT_EQUAL_UINT16(0, b.cellMilliVolts[9]);           // carried verbatim
  TEST_ASSERT_EQUAL_UINT32(0, b.cellPresentMask & (1u << 9));  // but flagged absent

  TEST_ASSERT_EQUAL_UINT8(15, b.liveCellCount);  // 16 carried, one of them dead
  TEST_ASSERT_EQUAL_UINT16(3330, b.cellMinMv);   // channel 1, not the 0 mV tap
  TEST_ASSERT_EQUAL_UINT16(3345, b.cellMaxMv);   // channel 16
  TEST_ASSERT_EQUAL_UINT16(15, b.cellSpreadMv);
  TEST_ASSERT_EQUAL_UINT8(0, b.cellMinIndex);
  TEST_ASSERT_EQUAL_UINT8(15, b.cellMaxIndex);
}

// A dropped reply must not become a packet full of confident zeros.
static void test_absent_field_groups_stay_clear(void) {
  jbd::BmsSnapshot s = {};
  s.valid    = true;
  s.fields   = jbd::kFieldPack;  // only the pack group arrived
  s.packVolts = 48.0f;
  s.currentAmps = 1.0f;

  BatteryTelemetry b = {};
  TEST_ASSERT_TRUE(toBatteryTelemetry(s, b));

  TEST_ASSERT_EQUAL_FLOAT(48.0f, b.packVolts);
  TEST_ASSERT_EQUAL_UINT16(kBatPack | kBatCurrent, b.fields);

  // Nothing else was read, so nothing else claims to be known.
  TEST_ASSERT_EQUAL_UINT16(0, b.fields & kBatMosfet);
  TEST_ASSERT_EQUAL_UINT16(0, b.fields & kBatProtection);
  TEST_ASSERT_EQUAL_UINT16(0, b.fields & kBatCells);
  TEST_ASSERT_EQUAL_UINT8(0, b.mosfetState);
  TEST_ASSERT_EQUAL_UINT32(0, b.cellPresentMask);
  TEST_ASSERT_EQUAL_UINT8(0, b.socPercent);
}

static void test_a_sixteen_channel_pack_fits_exactly(void) {
  jbd::BmsSnapshot s = benchPack();
  s.cellCount = 16;
  s.cellsReadMask &= 0xFFFFu;  // only 16 channels read
  BatteryTelemetry b = {};
  TEST_ASSERT_TRUE(toBatteryTelemetry(s, b));  // fits, so true
  TEST_ASSERT_EQUAL_UINT8(16, b.cellCount);
}

static void test_temperatures_become_tenths_with_a_presence_mask(void) {
  const jbd::BmsSnapshot s = benchPack();
  BatteryTelemetry       b = {};
  toBatteryTelemetry(s, b);

  TEST_ASSERT_EQUAL_UINT8(5, b.tempCount);
  TEST_ASSERT_EQUAL_INT16(250, b.tempDeciC[0]);
  TEST_ASSERT_EQUAL_INT16(290, b.tempDeciC[4]);
  TEST_ASSERT_EQUAL_UINT8(0x1F, b.tempPresentMask);  // five sensors
  TEST_ASSERT_EQUAL_INT16(250, b.tempMinDeciC);
  TEST_ASSERT_EQUAL_INT16(290, b.tempMaxDeciC);
  TEST_ASSERT_EQUAL_INT16(0, b.tempDeciC[5]);        // never reported
}

// ------------------------------- identity -----------------------------------

static void test_identity_carries_the_non_standard_obd_byte_verbatim(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.bitrate            = 500000;
  v.extendedAddressing = true;
  strcpy(v.vin, "ME4XXXXXXXXXXXXXX");
  strcpy(v.ecuName, "ECM -EngineControl");
  strcpy(v.calibrationId, "0105411002");
  v.cvn = 0xDEADBEEF;

  const uint8_t raw = 0x25;  // outside ISO 15031-5; this ECU really answers it
  obd::addSample(v, 0x1C, &raw, 1);

  IdentityTelemetry id = {};
  toIdentityTelemetry(v, id);

  TEST_ASSERT_EQUAL_STRING("ME4XXXXXXXXXXXXXX", id.vin);
  TEST_ASSERT_EQUAL_STRING("ECM -EngineControl", id.ecuName);
  TEST_ASSERT_EQUAL_STRING("0105411002", id.calibrationId);
  TEST_ASSERT_EQUAL_UINT32(0xDEADBEEF, id.cvn);
  TEST_ASSERT_EQUAL_UINT32(500000, id.ecuBitrate);
  TEST_ASSERT_EQUAL_UINT8(1, id.ecuExtendedAddressing);
  TEST_ASSERT_EQUAL_UINT8(0x25, id.obdStandardRaw);
  TEST_ASSERT_EQUAL_UINT16(kIdVehicle | kIdEcu, id.fields);
}

// A fixed-size char array must always be NUL terminated, whatever arrives.
static void test_identity_strings_are_always_terminated(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  memset(v.vin, 'A', sizeof(v.vin));  // no terminator at all

  IdentityTelemetry id = {};
  toIdentityTelemetry(v, id);
  TEST_ASSERT_EQUAL_UINT8(0, id.vin[sizeof(id.vin) - 1]);
  TEST_ASSERT_EQUAL_UINT(sizeof(id.vin) - 1, strlen(id.vin));
}

// -------------------------------- engine ------------------------------------

static obd::VehicleSnapshot sweepWithRpm(float rpm, float mapKpa) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.rpm    = rpm;
  v.mapKpa = mapKpa;
  return v;
}

static void test_engine_window_summarises_the_fast_channels(void) {
  EngineAccumulator acc;
  acc.reset(0);

  const float rpms[] = {1480, 1500, 1520, 1500, 1510};
  for (int i = 0; i < 5; ++i) {
    acc.noteRequest();
    acc.noteReply();
    acc.push(sweepWithRpm(rpms[i], 51.0f + i), static_cast<uint32_t>(i * 125));
  }

  EngineTelemetry e = {};
  TEST_ASSERT_TRUE(acc.flush(e, 1000));

  TEST_ASSERT_EQUAL_UINT16(5, e.rpm.samples);
  TEST_ASSERT_EQUAL_FLOAT(1480.0f, e.rpm.min);
  TEST_ASSERT_EQUAL_FLOAT(1520.0f, e.rpm.max);
  TEST_ASSERT_EQUAL_FLOAT(1502.0f, e.rpm.mean);
  TEST_ASSERT_EQUAL_FLOAT(1510.0f, e.rpm.last);
  TEST_ASSERT_EQUAL_UINT32(1000, e.windowMs);
  TEST_ASSERT_EQUAL_UINT16(5, e.requests);
  TEST_ASSERT_EQUAL_UINT16(5, e.replies);
  TEST_ASSERT_EQUAL_UINT16(kEngRpm | kEngMap, e.fields);

  // Channels that never answered are NaN with no samples, not zero.
  TEST_ASSERT_EQUAL_UINT16(0, e.throttlePercent.samples);
  TEST_ASSERT_TRUE(isnan(e.throttlePercent.mean));
  TEST_ASSERT_EQUAL_UINT16(0, e.fields & kEngThrottle);
}

static void test_flush_resets_the_window(void) {
  EngineAccumulator acc;
  acc.reset(0);
  acc.push(sweepWithRpm(1500, 50), 0);

  EngineTelemetry a = {};
  TEST_ASSERT_TRUE(acc.flush(a, 1000));

  // Second window sees only its own samples.
  acc.push(sweepWithRpm(3000, 90), 1100);
  EngineTelemetry b = {};
  TEST_ASSERT_TRUE(acc.flush(b, 2000));
  TEST_ASSERT_EQUAL_UINT16(1, b.rpm.samples);
  TEST_ASSERT_EQUAL_FLOAT(3000.0f, b.rpm.mean);
  TEST_ASSERT_EQUAL_UINT32(1000, b.windowMs);
}

static void test_an_empty_window_is_not_published(void) {
  EngineAccumulator acc;
  acc.reset(0);
  acc.noteRequest();  // asked, got nothing back

  EngineTelemetry e = {};
  memset(&e, 0xCD, sizeof(e));
  TEST_ASSERT_FALSE(acc.flush(e, 1000));

  // And the window restarted, so the next real sample is not mixed in with it.
  acc.push(sweepWithRpm(1500, 50), 1100);
  TEST_ASSERT_TRUE(acc.flush(e, 2000));
  TEST_ASSERT_EQUAL_UINT16(1, e.rpm.samples);
}

// -------------------------------- status ------------------------------------

static void test_first_status_reading_is_reported_without_claiming_an_edge(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.milOn          = true;
  v.storedDtcCount = 1;
  v.dtc[0]         = 0x0145;

  StatusTracker        t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_TRUE(t.update(v, st));

  TEST_ASSERT_EQUAL_UINT8(1, st.milOn);
  TEST_ASSERT_EQUAL_UINT8(1, st.dtcListed);
  TEST_ASSERT_EQUAL_UINT16(0x0145, st.dtc[0]);
  // No transition was observed, so none is claimed.
  TEST_ASSERT_EQUAL_UINT16(0, st.eventMask);
}

static void test_an_unchanged_state_publishes_nothing(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  v.storedDtcCount = 0;

  StatusTracker        t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_TRUE(t.update(v, st));   // the first one
  TEST_ASSERT_FALSE(t.update(v, st));  // identical, so silence
  TEST_ASSERT_FALSE(t.update(v, st));
}

static void test_a_new_dtc_raises_an_edge(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  StatusTracker        t;
  StatusEventTelemetry st = {};
  t.update(v, st);

  v.storedDtcCount = 1;
  v.dtc[0]         = 0x0301;
  TEST_ASSERT_TRUE(t.update(v, st));
  TEST_ASSERT_EQUAL_UINT16(kEvDtcAppeared, st.eventMask & kEvDtcAppeared);

  // Then it clears.
  v.storedDtcCount = 0;
  v.dtc[0]         = 0;
  TEST_ASSERT_TRUE(t.update(v, st));
  TEST_ASSERT_EQUAL_UINT16(kEvDtcCleared, st.eventMask & kEvDtcCleared);
}

static void test_the_mil_toggling_is_an_edge(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  StatusTracker        t;
  StatusEventTelemetry st = {};
  t.update(v, st);

  v.milOn = true;
  TEST_ASSERT_TRUE(t.update(v, st));
  TEST_ASSERT_EQUAL_UINT16(kEvMilChanged, st.eventMask & kEvMilChanged);
}

// Only a newly raised protection bit is a fault event. One that cleared is
// good news, and reporting it as a fault would train people to ignore the flag.
static void test_only_a_newly_raised_protection_bit_is_a_fault(void) {
  jbd::BmsSnapshot s = {};
  s.fields = jbd::kFieldProtect | jbd::kFieldMos;
  s.dischargeMosOn = true;

  StatusTracker        t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_TRUE(t.update(s, st));  // first reading

  s.protection = jbd::kCellUv;
  TEST_ASSERT_TRUE(t.update(s, st));
  TEST_ASSERT_EQUAL_UINT16(kEvBatteryFault, st.eventMask & kEvBatteryFault);
  TEST_ASSERT_EQUAL_UINT16(jbd::kCellUv, st.batteryProtectionFlags);

  s.protection = 0;  // cleared
  TEST_ASSERT_TRUE(t.update(s, st));
  TEST_ASSERT_EQUAL_UINT16(0, st.eventMask & kEvBatteryFault);
}

static void test_a_mosfet_change_is_an_edge(void) {
  jbd::BmsSnapshot s = {};
  s.fields = jbd::kFieldProtect | jbd::kFieldMos;
  s.dischargeMosOn = true;
  s.chargeMosOn    = true;

  StatusTracker        t;
  StatusEventTelemetry st = {};
  t.update(s, st);

  s.dischargeMosOn = false;  // the latched-off case
  TEST_ASSERT_TRUE(t.update(s, st));
  TEST_ASSERT_EQUAL_UINT16(kEvMosfetChanged, st.eventMask & kEvMosfetChanged);
  TEST_ASSERT_EQUAL_UINT8(kMosCharge, st.batteryMosfetState);
}

// A battery poll that read neither group says nothing, rather than reporting
// "no faults" on the strength of a reply that never arrived.
static void test_a_battery_poll_that_read_nothing_is_silent(void) {
  jbd::BmsSnapshot     s  = {};  // fields == 0
  StatusTracker        t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_FALSE(t.update(s, st));
}

static void test_forget_makes_the_next_reading_news_again(void) {
  obd::VehicleSnapshot v = {};
  obd::reset(v);
  StatusTracker        t;
  StatusEventTelemetry st = {};
  TEST_ASSERT_TRUE(t.update(v, st));
  TEST_ASSERT_FALSE(t.update(v, st));

  t.forget();
  TEST_ASSERT_TRUE(t.update(v, st));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_battery_mapping_carries_the_pack);
  RUN_TEST(test_the_dead_channel_is_marked_absent_not_zero);
  RUN_TEST(test_absent_field_groups_stay_clear);
  RUN_TEST(test_a_sixteen_channel_pack_fits_exactly);
  RUN_TEST(test_temperatures_become_tenths_with_a_presence_mask);
  RUN_TEST(test_identity_carries_the_non_standard_obd_byte_verbatim);
  RUN_TEST(test_identity_strings_are_always_terminated);
  RUN_TEST(test_engine_window_summarises_the_fast_channels);
  RUN_TEST(test_flush_resets_the_window);
  RUN_TEST(test_an_empty_window_is_not_published);
  RUN_TEST(test_first_status_reading_is_reported_without_claiming_an_edge);
  RUN_TEST(test_an_unchanged_state_publishes_nothing);
  RUN_TEST(test_a_new_dtc_raises_an_edge);
  RUN_TEST(test_the_mil_toggling_is_an_edge);
  RUN_TEST(test_only_a_newly_raised_protection_bit_is_a_fault);
  RUN_TEST(test_a_mosfet_change_is_an_edge);
  RUN_TEST(test_a_battery_poll_that_read_nothing_is_silent);
  RUN_TEST(test_forget_makes_the_next_reading_news_again);
  UNITY_END();
  return 0;
}
