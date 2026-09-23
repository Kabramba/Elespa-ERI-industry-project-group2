// jbd_protocol.h - pure encode/decode for the JBD protocols. No Arduino, no
// ESP-IDF, no I/O: every function here is a deterministic transform over bytes,
// which is what makes the whole layer testable on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bms_types.h"

namespace jbd {

// ============================ UART (protocol V4) ============================
// Request  : DD A5 <reg> 00 <chk_hi> <chk_lo> 77
// Response : DD <reg> <status> <len> <data...> <chk_hi> <chk_lo> 77
// Checksum : 0x10000 - sum(status/reg .. end of data)

constexpr uint8_t kStart    = 0xDD;
constexpr uint8_t kReadCmd  = 0xA5;
constexpr uint8_t kEnd      = 0x77;
constexpr uint8_t kRegBasic = 0x03;   // pack, protection, config, temperatures
constexpr uint8_t kRegCells = 0x04;   // per-cell millivolts
constexpr uint8_t kRegHwVer = 0x05;   // ASCII model string
constexpr size_t  kRequestBytes = 7;
constexpr size_t  kMaxFrameBytes = 4 + 255 + 3;

uint16_t checksum(uint8_t first, uint8_t len, const uint8_t *data);

// Writes exactly kRequestBytes into `out`. Returns the count written.
size_t buildReadRequest(uint8_t reg, uint8_t *out);

enum class ParseResult : int8_t {
  kOk = 0,
  kBadLength = -1,
  kBadEnd = -2,
  kBadChecksum = -3,
  kWrongRegister = -4,
  kBmsError = -5,
};
const char *parseResultName(ParseResult r);

struct Frame {
  uint8_t        reg;
  uint8_t        status;   // 0x00 ok, 0x80 error
  uint8_t        len;
  const uint8_t *data;     // points into the caller's buffer
};

ParseResult parseResponse(const uint8_t *buf, size_t n, uint8_t expectReg, Frame &out);

bool decodeBasic(const Frame &f, BmsSnapshot &s);   // register 0x03
bool decodeCells(const Frame &f, BmsSnapshot &s);   // register 0x04

// ================================== CAN =====================================
// Query based: the host sends a REMOTE frame, the BMS answers with a DATA frame
// of the same id. Payload is big-endian; the last two bytes are CRC-16/MODBUS,
// big-endian, over everything before them. Verified against live hardware.

constexpr uint32_t kCanPack      = 0x100;  // pack V, current, remaining
constexpr uint32_t kCanCapacity  = 0x101;  // full capacity, cycles, RSOC
constexpr uint32_t kCanBalProt   = 0x102;  // balance bits, protection flags
constexpr uint32_t kCanMosDate   = 0x103;  // MOS status, mfg date, sw version
constexpr uint32_t kCanConfig    = 0x104;  // cell count, NTC count (DLC 4)
constexpr uint32_t kCanNtc1      = 0x105;
constexpr uint32_t kCanNtc4      = 0x106;
constexpr uint32_t kCanCellFirst = 0x107;  // three cells per frame
constexpr uint32_t kCanCellLast  = 0x110;

uint16_t crc16Modbus(const uint8_t *p, size_t n);

// Frames shorter than 3 bytes carry no CRC and pass trivially.
bool canCrcOk(const uint8_t *data, uint8_t dlc);

// Decodes one reply into `s`. Returns false if the id is unknown or the frame
// is too short for its documented contents. A DLC of 0 is a legitimate "that
// range holds no cells" answer and decodes to nothing, returning true.
bool decodeCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc, BmsSnapshot &s);

// How many 0x107.. frames are needed for `cellCount` cells.
uint8_t canCellFrameCount(uint8_t cellCount);

}  // namespace jbd
