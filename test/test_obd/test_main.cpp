// The transmit guard is the safety-critical part of talking to a vehicle ECU,
// so it is tested exhaustively on the host before any frame reaches the bus.
#include <initializer_list>

#include <unity.h>

#include "obd/obd_isotp.h"

using namespace obd;

void setUp(void) {}
void tearDown(void) {}

static void test_only_read_only_services_are_allowed(void) {
  // Permitted: live data, freeze frame, DTC reads, vehicle info.
  for (uint8_t sid : {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A})
    TEST_ASSERT_TRUE_MESSAGE(isAllowedSid(sid), "should be allowed");

  // Refused: everything that changes ECU state or writes into it.
  for (uint8_t sid : {0x04, 0x08, 0x10, 0x11, 0x14, 0x22, 0x27, 0x28, 0x2E, 0x2F,
                      0x31, 0x34, 0x35, 0x36, 0x37, 0x3D, 0x3E, 0x85, 0x19})
    TEST_ASSERT_FALSE_MESSAGE(isAllowedSid(sid), "must be refused");
}

static void test_clear_dtc_request_is_refused(void) {
  // 04 is "clear DTCs" - wiping a rider's fault history. Must never go out.
  const uint8_t clear[8] = {0x01, 0x04, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(clear, 8));

  const uint8_t reset[8] = {0x02, 0x11, 0x01, 0, 0, 0, 0, 0};  // ECU reset
  TEST_ASSERT_FALSE(isPermittedRequestFrame(reset, 8));

  const uint8_t session[8] = {0x02, 0x10, 0x03, 0, 0, 0, 0, 0};  // session control
  TEST_ASSERT_FALSE(isPermittedRequestFrame(session, 8));
}

static void test_valid_read_request_is_permitted(void) {
  const uint8_t pid00[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};  // supported PIDs
  TEST_ASSERT_TRUE(isPermittedRequestFrame(pid00, 8));

  const uint8_t vin[8] = {0x02, 0x09, 0x02, 0, 0, 0, 0, 0};    // VIN
  TEST_ASSERT_TRUE(isPermittedRequestFrame(vin, 8));

  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};     // Flow Control
  TEST_ASSERT_TRUE(isPermittedRequestFrame(fc, 8));
}

// A multi-frame request means pushing a payload into the ECU. Refused whatever
// SID it claims.
static void test_multiframe_requests_are_refused(void) {
  const uint8_t first[8] = {0x10, 0x14, 0x01, 0x00, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(first, 8));

  const uint8_t consec[8] = {0x21, 0x01, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(consec, 8));
}

static void test_malformed_frames_are_refused(void) {
  const uint8_t good[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(good, 7));   // wrong DLC
  TEST_ASSERT_FALSE(isPermittedRequestFrame(nullptr, 8));

  const uint8_t zeroLen[8] = {0x00, 0x01, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(zeroLen, 8));

  const uint8_t badPci[8] = {0x40, 0x01, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(isPermittedRequestFrame(badPci, 8));
}

static void test_builders_produce_exactly_what_the_reference_shows(void) {
  uint8_t       out[8];
  const uint8_t pid = 0x00;
  TEST_ASSERT_EQUAL_UINT(8, buildSingleFrame(0x01, &pid, 1, out));
  // docs/activa-can.md: 7DF 02 01 00 00 00 00 00 00
  const uint8_t expect[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, out, 8);
  TEST_ASSERT_TRUE(isPermittedRequestFrame(out, 8));

  TEST_ASSERT_EQUAL_UINT(8, buildFlowControl(out));
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fc, out, 8);
}

static void test_pci_decoding(void) {
  const uint8_t sf[8] = {0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0x00};
  TEST_ASSERT_EQUAL_INT((int)Pci::kSingle, (int)pciOf(sf, 8));
  TEST_ASSERT_EQUAL_UINT8(6, singleFrameLength(sf));

  const uint8_t ff[8] = {0x10, 0x14, 0x49, 0x02, 0x01, 0x4D, 0x41, 0x31};
  TEST_ASSERT_EQUAL_INT((int)Pci::kFirst, (int)pciOf(ff, 8));
  TEST_ASSERT_EQUAL_UINT16(0x14, firstFrameLength(ff));  // 20 bytes, e.g. a VIN

  const uint8_t cf[8] = {0x21, 0x4A, 0x5A, 0x33, 0x31, 0x32, 0x33, 0x34};
  TEST_ASSERT_EQUAL_INT((int)Pci::kConsecutive, (int)pciOf(cf, 8));
  TEST_ASSERT_EQUAL_UINT8(1, consecutiveIndex(cf));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_only_read_only_services_are_allowed);
  RUN_TEST(test_clear_dtc_request_is_refused);
  RUN_TEST(test_valid_read_request_is_permitted);
  RUN_TEST(test_multiframe_requests_are_refused);
  RUN_TEST(test_malformed_frames_are_refused);
  RUN_TEST(test_builders_produce_exactly_what_the_reference_shows);
  RUN_TEST(test_pci_decoding);
  UNITY_END();
  return 0;
}
