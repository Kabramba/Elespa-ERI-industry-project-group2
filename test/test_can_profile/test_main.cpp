// Host-side tests for the CAN profile layer: bit-timing arithmetic, profile
// validation rules, and the registry. No hardware.
#include <string.h>

#include <initializer_list>

#include <unity.h>

#include "can/active_profile.h"
#include "can/can_bus_profile.h"
#include "can/can_profile_registry.h"
#include "can/can_timing.h"

using namespace can;

void setUp(void) {}
void tearDown(void) {}

// ------------------------------- timing -------------------------------------

// The solver must reproduce ESP-IDF's own canonical configs at 80 MHz. If it
// does, the arithmetic is right for every other rate too.
static void test_timing_matches_esp_idf_canonical_configs(void) {
  struct Known { uint32_t bitrate; uint32_t brp; uint8_t t1, t2; };
  const Known kKnown[] = {
      {1000000, 4,  15, 4},   // TWAI_TIMING_CONFIG_1MBITS
      {500000,  8,  15, 4},   // TWAI_TIMING_CONFIG_500KBITS
      {250000,  16, 15, 4},   // TWAI_TIMING_CONFIG_250KBITS
      {125000,  32, 15, 4},   // TWAI_TIMING_CONFIG_125KBITS
  };
  for (const Known &k : kKnown) {
    BitTiming t;
    TEST_ASSERT_TRUE_MESSAGE(computeBitTiming(k.bitrate, 80, t), "no solution");
    TEST_ASSERT_EQUAL_UINT32(k.brp, t.brp);
    TEST_ASSERT_EQUAL_UINT8(k.t1, t.tseg1);
    TEST_ASSERT_EQUAL_UINT8(k.t2, t.tseg2);
    TEST_ASSERT_EQUAL_UINT32(k.bitrate, t.actualBitrate);
    TEST_ASSERT_EQUAL_UINT8(80, t.actualSamplePointPercent);
  }
}

static void test_timing_honours_a_later_sample_point(void) {
  BitTiming t;
  TEST_ASSERT_TRUE(computeBitTiming(500000, 90, t));
  TEST_ASSERT_EQUAL_UINT32(500000, t.actualBitrate);
  TEST_ASSERT_TRUE(t.actualSamplePointPercent >= 85);
  // Whatever it picks must stay inside the register limits.
  TEST_ASSERT_TRUE(t.tseg1 >= kTseg1Min && t.tseg1 <= kTseg1Max);
  TEST_ASSERT_TRUE(t.tseg2 >= kTseg2Min && t.tseg2 <= kTseg2Max);
  TEST_ASSERT_TRUE(t.sjw >= 1 && t.sjw <= kSjwMax);
  TEST_ASSERT_TRUE(t.sjw <= t.tseg2);
  TEST_ASSERT_EQUAL_UINT32(0, t.brp % 2);
}

// Approximate bitrates desynchronise a bus, so an inexact rate must fail
// outright rather than silently round.
static void test_timing_refuses_unreachable_bitrates(void) {
  BitTiming t;
  TEST_ASSERT_FALSE(computeBitTiming(333333, 80, t));
  TEST_ASSERT_FALSE(computeBitTiming(0, 80, t));
  TEST_ASSERT_FALSE(computeBitTiming(80000000, 80, t));  // 1 tq per bit
}

// ------------------------------- profiles -----------------------------------

static CanBusProfile makeValid() {
  CanBusProfile p{};
  p.device.key         = "test";
  p.device.displayName = "test";
  p.bus.bitrate        = 500000;
  p.bus.txPin          = 5;
  p.bus.rxPin          = 4;
  p.bus.frameFormat    = FrameFormat::kStandard11;
  p.protocol.interaction = Interaction::kRemoteFrameQuery;
  p.protocol.idFirst     = 0x100;
  p.protocol.idLast      = 0x110;
  p.protocol.retries     = 2;
  p.access               = AccessPolicy::kReadOnlyQueries;
  // A profile permitted to transmit must declare what; an empty allowlist is
  // now a validation error, which is the point of the mechanism.
  p.txAllow.rules[0] = {0x100, 0x110, 0};
  p.txAllow.count    = 1;
  return p;
}

static void test_valid_profile_passes(void) {
  const char *why = nullptr;
  TEST_ASSERT_TRUE_MESSAGE(validate(makeValid(), &why), why ? why : "");
}

static void test_validation_catches_bad_profiles(void) {
  const char *why;
  CanBusProfile p;

  p = makeValid(); p.bus.txPin = -1;
  TEST_ASSERT_FALSE(validate(p, &why));

  p = makeValid(); p.bus.rxPin = p.bus.txPin;
  TEST_ASSERT_FALSE(validate(p, &why));

  p = makeValid(); p.bus.bitrate = 333333;
  TEST_ASSERT_FALSE(validate(p, &why));

  // An 11-bit profile cannot poll a 29-bit id range.
  p = makeValid(); p.protocol.idLast = 0x18904001;
  TEST_ASSERT_FALSE(validate(p, &why));

  p = makeValid(); p.protocol.retries = 0;
  TEST_ASSERT_FALSE(validate(p, &why));

  // ISO-TP needs both ends of the conversation.
  p = makeValid();
  p.protocol.interaction = Interaction::kIsoTp;
  p.protocol.requestId   = 0x7DF;
  p.protocol.responseId  = 0;
  TEST_ASSERT_FALSE(validate(p, &why));
}

// The safety rule that matters most: a passive profile must not be able to put
// anything on somebody else's bus.
static void test_listen_only_can_never_transmit(void) {
  CanBusProfile p = makeValid();

  p.access = AccessPolicy::kListenOnly;
  TEST_ASSERT_FALSE(allowsTransmit(p));

  // The access policy alone is enough...
  p.access   = AccessPolicy::kReadWrite;
  p.bus.mode = BusMode::kListenOnly;
  TEST_ASSERT_FALSE(allowsTransmit(p));  // ...and so is the bus mode

  p.bus.mode = BusMode::kNormal;
  TEST_ASSERT_TRUE(allowsTransmit(p));
}

static void test_broadcast_profile_must_not_transmit(void) {
  const char   *why;
  CanBusProfile p     = makeValid();
  p.protocol.interaction = Interaction::kBroadcastListen;
  p.access               = AccessPolicy::kReadWrite;  // contradiction
  TEST_ASSERT_FALSE(validate(p, &why));
}

static void test_id_range_and_format(void) {
  const CanBusProfile p = makeValid();
  TEST_ASSERT_TRUE(respondsOnId(p, 0x104));
  TEST_ASSERT_FALSE(respondsOnId(p, 0x0FF));
  TEST_ASSERT_FALSE(respondsOnId(p, 0x111));
  TEST_ASSERT_TRUE(idFitsFormat(p, 0x7FF));
  TEST_ASSERT_FALSE(idFitsFormat(p, 0x800));  // needs extended framing
}

// ------------------------------- registry -----------------------------------

static void test_registry_entries_are_all_valid(void) {
  const char *badKey = nullptr, *reason = nullptr;
  const size_t bad = validateAll(&badKey, &reason);
  if (bad) TEST_FAIL_MESSAGE(reason ? reason : "a profile failed validation");
  TEST_ASSERT_EQUAL_UINT(0, bad);
  TEST_ASSERT_TRUE(profileCount() >= 3);
}

static void test_registry_lookup(void) {
  TEST_ASSERT_NULL(findProfile("nope"));
  TEST_ASSERT_NULL(findProfile(nullptr));
  TEST_ASSERT_NULL(profileAt(profileCount()));
  TEST_ASSERT_NOT_NULL(profileAt(0));

  const CanBusProfile *bms = findByClass(DeviceClass::kBatteryBms);
  TEST_ASSERT_NOT_NULL(bms);
  TEST_ASSERT_EQUAL_STRING("jbd-sp24s", bms->device.key);
}

// Pins the measured JBD values so a careless edit to the table fails here.
static void test_jbd_profile_matches_what_was_measured(void) {
  const CanBusProfile *p = findProfile("jbd-sp24s");
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_EQUAL_UINT32(500000, p->bus.bitrate);
  TEST_ASSERT_EQUAL_INT8(5, p->bus.txPin);   // opposite of the vendor notes
  TEST_ASSERT_EQUAL_INT8(4, p->bus.rxPin);
  TEST_ASSERT_TRUE(p->bus.hardwareRetransmit);
  TEST_ASSERT_EQUAL_INT((int)FrameFormat::kStandard11, (int)p->bus.frameFormat);
  TEST_ASSERT_EQUAL_INT((int)Interaction::kRemoteFrameQuery, (int)p->protocol.interaction);
  TEST_ASSERT_EQUAL_INT((int)CrcScheme::kModbusBigEndian, (int)p->protocol.crc);
  TEST_ASSERT_EQUAL_UINT32(0x100, p->protocol.idFirst);
  TEST_ASSERT_EQUAL_UINT32(0x110, p->protocol.idLast);
  TEST_ASSERT_EQUAL_INT((int)AccessPolicy::kReadOnlyQueries, (int)p->access);
  TEST_ASSERT_EQUAL_INT((int)Provenance::kMeasured, (int)p->device.provenance);
}

// The sniffers exist to approach an unknown bus safely; if one could ever
// transmit it would defeat the point.
static void test_sniffer_profiles_are_passive(void) {
  for (const char *key : {"sniff-500k", "sniff-250k"}) {
    const CanBusProfile *p = findProfile(key);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_FALSE(allowsTransmit(*p));
    TEST_ASSERT_EQUAL_INT((int)BusMode::kListenOnly, (int)p->bus.mode);
    TEST_ASSERT_EQUAL_INT((int)AccessPolicy::kListenOnly, (int)p->access);
  }
}

// ------------------------- transmit allowlist -------------------------------

// The guard the Activa reference calls non-negotiable: every transmit goes
// through mayTransmitId(), and anything not explicitly listed is refused.
static void test_obd11_transmit_allowlist(void) {
  const CanBusProfile *p = findProfile("activa-obd11");
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x7DF));   // functional request
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x7E0));   // physical request
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x7E7));
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x7E8));  // that is the ECU's reply id
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x123));  // anything else on the vehicle bus
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x000));
}

// 0x18DAxxF1 cannot be expressed as a plain range: the low byte is fixed, so
// the mask has to do the work.
static void test_obd29_mask_pins_the_fixed_bytes(void) {
  const CanBusProfile *p = findProfile("activa-obd29");
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x18DB33F1));   // functional
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x18DA10F1));   // physical to ECU 0x10
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x18DA33F1));
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x18DA1055));  // in range, wrong low byte
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x18DAF110));  // that is a response id
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x18FF00F1));  // wrong high half
}

// A passive profile must refuse every id, with no exceptions and no ordering
// dependence on how it was configured.
static void test_listen_profiles_transmit_nothing(void) {
  for (const char *key : {"activa-listen", "sniff-500k", "sniff-250k", "sniff-1m", "sniff-125k"}) {
    const CanBusProfile *p = findProfile(key);
    TEST_ASSERT_NOT_NULL_MESSAGE(p, key);
    TEST_ASSERT_EQUAL_UINT8(0, p->txAllow.count);
    TEST_ASSERT_FALSE(allowsTransmit(*p));
    TEST_ASSERT_FALSE(mayTransmitId(*p, 0x7DF));
    TEST_ASSERT_FALSE(mayTransmitId(*p, 0x000));
    TEST_ASSERT_FALSE(mayTransmitId(*p, 0x1FFFFFFF));
  }
}

// An empty allowlist refuses everything even if the policy says otherwise.
static void test_empty_allowlist_fails_closed(void) {
  CanBusProfile p = makeValid();
  p.access        = AccessPolicy::kReadWrite;
  p.bus.mode      = BusMode::kNormal;
  p.txAllow.count = 0;
  TEST_ASSERT_TRUE(allowsTransmit(p));
  TEST_ASSERT_FALSE(mayTransmitId(p, 0x100));  // fail closed
  const char *why;
  TEST_ASSERT_FALSE(validate(p, &why));        // and it is a config error
}

static void test_jbd_allowlist_covers_only_its_ids(void) {
  const CanBusProfile *p = findProfile("jbd-sp24s");
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x100));
  TEST_ASSERT_TRUE(mayTransmitId(*p, 0x110));
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x0FF));
  TEST_ASSERT_FALSE(mayTransmitId(*p, 0x111));
}

// --------------------------- Activa profiles --------------------------------

static void test_activa_profiles_are_present_and_honest(void) {
  for (const char *key : {"activa-listen", "activa-obd11", "activa-obd29"}) {
    const CanBusProfile *p = findProfile(key);
    TEST_ASSERT_NOT_NULL_MESSAGE(p, key);
    // The bitrate is still a guess, so nothing here may claim to be measured.
    TEST_ASSERT_EQUAL_INT((int)Provenance::kUnverified, (int)p->device.provenance);
    TEST_ASSERT_EQUAL_INT((int)DeviceClass::kVehicleEcu, (int)p->device.deviceClass);
    // The vehicle carries one terminator; the module's 120R must stay fitted.
    TEST_ASSERT_TRUE(p->bus.expectsExternalTermination);
  }

  const CanBusProfile *o11 = findProfile("activa-obd11");
  TEST_ASSERT_EQUAL_INT((int)Interaction::kIsoTp, (int)o11->protocol.interaction);
  TEST_ASSERT_EQUAL_INT((int)FrameFormat::kStandard11, (int)o11->bus.frameFormat);
  TEST_ASSERT_EQUAL_UINT32(0x7DF, o11->protocol.requestId);
  TEST_ASSERT_EQUAL_UINT32(0x7E8, o11->protocol.responseId);

  const CanBusProfile *o29 = findProfile("activa-obd29");
  TEST_ASSERT_EQUAL_INT((int)FrameFormat::kExtended29, (int)o29->bus.frameFormat);
  TEST_ASSERT_EQUAL_UINT32(0x18DB33F1, o29->protocol.requestId);
  TEST_ASSERT_EQUAL_UINT32(0x18DAF110, o29->protocol.responseId);
}

// Every candidate bitrate in the probe order must actually be reachable.
static void test_every_probe_bitrate_resolves(void) {
  for (const char *key : {"sniff-500k", "sniff-250k", "sniff-1m", "sniff-125k"}) {
    const CanBusProfile *p = findProfile(key);
    TEST_ASSERT_NOT_NULL_MESSAGE(p, key);
    BitTiming t;
    TEST_ASSERT_TRUE_MESSAGE(resolveTiming(*p, t), key);
    TEST_ASSERT_EQUAL_UINT32(p->bus.bitrate, t.actualBitrate);
  }
}

// ------------------------- runtime selection --------------------------------

static void test_active_profile_is_runtime_selectable(void) {
  TEST_ASSERT_EQUAL_STRING("jbd-sp24s", active().device.key);  // first entry

  TEST_ASSERT_TRUE(setActive("activa-listen"));
  TEST_ASSERT_EQUAL_STRING("activa-listen", active().device.key);
  TEST_ASSERT_TRUE(activeWasChosen());

  TEST_ASSERT_FALSE(setActive("does-not-exist"));
  TEST_ASSERT_EQUAL_STRING("activa-listen", active().device.key);  // unchanged

  TEST_ASSERT_FALSE(setActiveIndex(profileCount()));
  TEST_ASSERT_TRUE(setActiveIndex(0));
  TEST_ASSERT_EQUAL_STRING("jbd-sp24s", active().device.key);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_timing_matches_esp_idf_canonical_configs);
  RUN_TEST(test_timing_honours_a_later_sample_point);
  RUN_TEST(test_timing_refuses_unreachable_bitrates);
  RUN_TEST(test_valid_profile_passes);
  RUN_TEST(test_validation_catches_bad_profiles);
  RUN_TEST(test_listen_only_can_never_transmit);
  RUN_TEST(test_broadcast_profile_must_not_transmit);
  RUN_TEST(test_id_range_and_format);
  RUN_TEST(test_registry_entries_are_all_valid);
  RUN_TEST(test_registry_lookup);
  RUN_TEST(test_jbd_profile_matches_what_was_measured);
  RUN_TEST(test_sniffer_profiles_are_passive);
  RUN_TEST(test_obd11_transmit_allowlist);
  RUN_TEST(test_obd29_mask_pins_the_fixed_bytes);
  RUN_TEST(test_listen_profiles_transmit_nothing);
  RUN_TEST(test_empty_allowlist_fails_closed);
  RUN_TEST(test_jbd_allowlist_covers_only_its_ids);
  RUN_TEST(test_activa_profiles_are_present_and_honest);
  RUN_TEST(test_every_probe_bitrate_resolves);
  RUN_TEST(test_active_profile_is_runtime_selectable);
  UNITY_END();
  return 0;
}
