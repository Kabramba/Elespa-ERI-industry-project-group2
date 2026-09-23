// Host-side tests for the pure JBD decode layer.
//
// Fixtures are REAL frames: the documented reference frame from the protocol
// spec, plus frames captured off this actual pack on 2026-09-21/23. If a decode
// regression ever creeps in, these fail on the laptop in under a second with no
// hardware attached.
#include <string.h>
#include <unity.h>

#include "bms_types.h"
#include "jbd_protocol.h"

using namespace jbd;

void setUp(void) {}
void tearDown(void) {}

// ------------------------------ UART ----------------------------------------

static void test_request_checksum_matches_spec(void) {
  uint8_t req[kRequestBytes];
  TEST_ASSERT_EQUAL_UINT(kRequestBytes, buildReadRequest(kRegBasic, req));
  // DD A5 03 00 FF FD 77 - the exact bytes the working sketch put on the wire.
  const uint8_t expect[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, req, 7);
  TEST_ASSERT_EQUAL_HEX16(0xFFFD, checksum(kRegBasic, 0, nullptr));
}

// Reference frame from the JBD V4 spec: 58.88 V, 7.20/10.00 Ah, 72 %,
// 15 cells, 2 NTC at 20.3 / 21.5 C.
static const uint8_t kSpecBasic[] = {
    0xDD, 0x03, 0x00, 0x1B,
    0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B,
    0x76, 0x0B, 0x82,
    0xFB, 0xFF, 0x77};

static void test_parse_and_decode_spec_frame(void) {
  Frame f;
  TEST_ASSERT_EQUAL_INT((int)ParseResult::kOk,
                        (int)parseResponse(kSpecBasic, sizeof(kSpecBasic), kRegBasic, f));

  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeBasic(f, s));
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 58.88f, s.packVolts);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, s.currentAmps);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 7.20f, s.remainingAh);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 10.00f, s.fullAh);
  TEST_ASSERT_EQUAL_UINT8(72, s.socPercent);
  TEST_ASSERT_EQUAL_UINT8(15, s.cellCount);
  TEST_ASSERT_EQUAL_UINT8(2, s.ntcCount);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 20.3f, s.ntcCelsius[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 21.5f, s.ntcCelsius[1]);
  TEST_ASSERT_TRUE(s.chargeMosOn);
  TEST_ASSERT_TRUE(s.dischargeMosOn);
  TEST_ASSERT_EQUAL_INT((int)Source::kUart, (int)s.source);
  TEST_ASSERT_TRUE(s.valid);
}

static void test_corrupt_frames_are_rejected(void) {
  uint8_t bad[sizeof(kSpecBasic)];
  Frame f;

  memcpy(bad, kSpecBasic, sizeof(bad));
  bad[10] ^= 0xFF;  // flip a payload byte
  TEST_ASSERT_EQUAL_INT((int)ParseResult::kBadChecksum,
                        (int)parseResponse(bad, sizeof(bad), kRegBasic, f));

  memcpy(bad, kSpecBasic, sizeof(bad));
  bad[sizeof(bad) - 1] = 0x00;  // clobber the terminator
  TEST_ASSERT_EQUAL_INT((int)ParseResult::kBadEnd,
                        (int)parseResponse(bad, sizeof(bad), kRegBasic, f));

  TEST_ASSERT_EQUAL_INT((int)ParseResult::kWrongRegister,
                        (int)parseResponse(kSpecBasic, sizeof(kSpecBasic), kRegCells, f));

  TEST_ASSERT_EQUAL_INT((int)ParseResult::kBadLength,
                        (int)parseResponse(kSpecBasic, 5, kRegBasic, f));
}

// ------------------------------- CAN ----------------------------------------

static void test_crc16_modbus_check_value(void) {
  const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x4B37, crc16Modbus(v, sizeof(v)));  // standard check value
}

// Captured live. 0x104 -> 17 cells, 5 NTC. CRC is big-endian E3 CD.
static void test_live_can_config_frame(void) {
  const uint8_t f104[4] = {0x11, 0x05, 0xE3, 0xCD};
  TEST_ASSERT_TRUE(canCrcOk(f104, 4));

  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeCanFrame(kCanConfig, f104, 4, s));
  TEST_ASSERT_EQUAL_UINT8(17, s.cellCount);
  TEST_ASSERT_EQUAL_UINT8(5, s.ntcCount);
}

static void test_live_can_pack_and_capacity(void) {
  const uint8_t f100[8] = {0x16, 0x50, 0x00, 0x00, 0x00, 0x00, 0xE1, 0xC2};
  const uint8_t f101[8] = {0x11, 0x94, 0x00, 0x00, 0x00, 0x00, 0x87, 0x32};
  const uint8_t f102[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xDA, 0x81};
  TEST_ASSERT_TRUE(canCrcOk(f100, 8));
  TEST_ASSERT_TRUE(canCrcOk(f101, 8));
  TEST_ASSERT_TRUE(canCrcOk(f102, 8));

  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeCanFrame(kCanPack, f100, 8, s));
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 57.12f, s.packVolts);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, s.currentAmps);

  TEST_ASSERT_TRUE(decodeCanFrame(kCanCapacity, f101, 8, s));
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 45.00f, s.fullAh);
  TEST_ASSERT_EQUAL_UINT16(0, s.cycles);
  TEST_ASSERT_EQUAL_UINT8(0, s.socPercent);

  TEST_ASSERT_TRUE(decodeCanFrame(kCanBalProt, f102, 8, s));
  TEST_ASSERT_EQUAL_HEX16(kCellUv, s.protection);  // the latched undervoltage
  TEST_ASSERT_EQUAL_UINT32(0, s.balanceBits);
}

// 0x106 arrives as DLC 6 because the pack has five NTCs, not six. Decoding a
// fixed six payload bytes would read the CRC as a temperature - the bug the
// first capability sweep printed as "T6=1497.0C".
static void test_short_frames_do_not_read_crc_as_data(void) {
  const uint8_t f105[8] = {0x0B, 0xD7, 0x0B, 0xBC, 0x0B, 0xBF, 0x32, 0xF0};
  const uint8_t f106[6] = {0x0B, 0xBE, 0x0B, 0xC3, 0x45, 0x25};

  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeCanFrame(kCanNtc1, f105, 8, s));
  TEST_ASSERT_TRUE(decodeCanFrame(kCanNtc4, f106, 6, s));

  TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, s.ntcCelsius[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 27.3f, s.ntcCelsius[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 27.6f, s.ntcCelsius[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 27.5f, s.ntcCelsius[3]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 28.0f, s.ntcCelsius[4]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.ntcCelsius[5]);  // never written
}

// Out-of-range cell ids answer with DLC 0 rather than staying silent.
static void test_empty_cell_frame_is_accepted(void) {
  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeCanFrame(0x10E, nullptr, 0, s));
  TEST_ASSERT_EQUAL_UINT16(0, s.cellMilliVolts[0]);
}

static void test_cell_frame_mapping(void) {
  const uint8_t f107[8] = {0x0D, 0xF6, 0x0D, 0xF1, 0x0D, 0xF3, 0x99, 0xDE};
  TEST_ASSERT_TRUE(canCrcOk(f107, 8));

  BmsSnapshot s = {};
  TEST_ASSERT_TRUE(decodeCanFrame(kCanCellFirst, f107, 8, s));
  TEST_ASSERT_EQUAL_UINT16(3574, s.cellMilliVolts[0]);
  TEST_ASSERT_EQUAL_UINT16(3569, s.cellMilliVolts[1]);
  TEST_ASSERT_EQUAL_UINT16(3571, s.cellMilliVolts[2]);

  TEST_ASSERT_EQUAL_UINT8(6, canCellFrameCount(17));
  TEST_ASSERT_EQUAL_UINT8(1, canCellFrameCount(3));
  TEST_ASSERT_EQUAL_UINT8(2, canCellFrameCount(4));
}

// --------------------------- derived stats ----------------------------------

// The real pack: 17 configured channels, channel 10 an unpopulated bridged tap.
static void test_derived_stats_ignore_the_dead_channel(void) {
  BmsSnapshot s = {};
  const uint16_t cells[17] = {3574, 3569, 3572, 3571, 3573, 3572, 3573, 3568, 3567,
                              0,    3568, 3569, 3571, 3568, 3571, 3568, 3573};
  memcpy(s.cellMilliVolts, cells, sizeof(cells));
  s.cellCount     = 17;
  s.cellsReadMask = 0x1FFFF;  // all 17 channels genuinely read
  computeDerived(s);

  TEST_ASSERT_EQUAL_UINT8(16, s.liveCellCount);       // not 17
  TEST_ASSERT_EQUAL_UINT16(3567, s.cellMinMv);        // not 0
  TEST_ASSERT_EQUAL_UINT16(3574, s.cellMaxMv);
  TEST_ASSERT_EQUAL_UINT16(7, s.cellSpreadMv);        // a healthy pack
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 57.127f, s.cellSumVolts);
}

// A dropped frame must be distinguishable from a genuine zero.
static void test_field_mask_marks_what_arrived(void) {
  BmsSnapshot s = {};
  TEST_ASSERT_EQUAL_UINT16(0, s.fields);
  TEST_ASSERT_FALSE(hasFields(s, kFieldPack));

  const uint8_t f100[8] = {0x16, 0x50, 0x00, 0x00, 0x00, 0x00, 0xE1, 0xC2};
  decodeCanFrame(kCanPack, f100, 8, s);
  TEST_ASSERT_TRUE(hasFields(s, kFieldPack));
  // 0x103 never arrived, so the MOS state must NOT read as a valid "off".
  TEST_ASSERT_FALSE(hasFields(s, kFieldMos));
  TEST_ASSERT_FALSE(hasFields(s, kFieldProtect));

  const uint8_t f103[8] = {0x00, 0x01, 0x33, 0x53, 0x00, 0x60, 0x00, 0x00};
  decodeCanFrame(kCanMosDate, f103, 8, s);
  TEST_ASSERT_TRUE(hasFields(s, kFieldMos));
  TEST_ASSERT_TRUE(s.chargeMosOn);
  TEST_ASSERT_FALSE(s.dischargeMosOn);
  TEST_ASSERT_EQUAL_UINT16(2025, s.mfgYear);
  TEST_ASSERT_EQUAL_UINT8(10, s.mfgMonth);
  TEST_ASSERT_EQUAL_UINT8(19, s.mfgDay);
  TEST_ASSERT_EQUAL_HEX16(0x0060, s.softwareVersion);
}

// Cells that were never received must not be folded into the statistics.
static void test_partial_cell_read_excludes_missing(void) {
  BmsSnapshot s = {};
  s.cellCount = 17;
  const uint8_t f107[8] = {0x0D, 0xF6, 0x0D, 0xF1, 0x0D, 0xF3, 0x99, 0xDE};
  decodeCanFrame(kCanCellFirst, f107, 8, s);   // cells 1-3 only
  computeDerived(s);

  TEST_ASSERT_EQUAL_UINT8(3, s.cellsRead);      // not 17
  TEST_ASSERT_EQUAL_UINT8(3, s.liveCellCount);
  TEST_ASSERT_EQUAL_UINT16(3569, s.cellMinMv);  // not 0 from the unread channels
  TEST_ASSERT_EQUAL_UINT16(3574, s.cellMaxMv);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 10.714f, s.cellSumVolts);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_request_checksum_matches_spec);
  RUN_TEST(test_parse_and_decode_spec_frame);
  RUN_TEST(test_corrupt_frames_are_rejected);
  RUN_TEST(test_crc16_modbus_check_value);
  RUN_TEST(test_live_can_config_frame);
  RUN_TEST(test_live_can_pack_and_capacity);
  RUN_TEST(test_short_frames_do_not_read_crc_as_data);
  RUN_TEST(test_empty_cell_frame_is_accepted);
  RUN_TEST(test_cell_frame_mapping);
  RUN_TEST(test_derived_stats_ignore_the_dead_channel);
  RUN_TEST(test_field_mask_marks_what_arrived);
  RUN_TEST(test_partial_cell_read_excludes_missing);
  return UNITY_END();
}
