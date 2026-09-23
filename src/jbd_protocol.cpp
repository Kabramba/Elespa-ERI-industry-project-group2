#include "jbd_protocol.h"

#include <string.h>

namespace jbd {
namespace {
inline uint16_t be16(const uint8_t *p) { return static_cast<uint16_t>((uint16_t(p[0]) << 8) | p[1]); }
}  // namespace

// ================================== UART ====================================

uint16_t checksum(uint8_t first, uint8_t len, const uint8_t *data) {
  uint16_t c = 0;
  c -= first;
  c -= len;
  for (uint16_t i = 0; i < len; ++i) c -= data[i];
  return c;
}

size_t buildReadRequest(uint8_t reg, uint8_t *out) {
  const uint16_t c = checksum(reg, 0, nullptr);
  out[0] = kStart;
  out[1] = kReadCmd;
  out[2] = reg;
  out[3] = 0x00;
  out[4] = static_cast<uint8_t>(c >> 8);
  out[5] = static_cast<uint8_t>(c & 0xFF);
  out[6] = kEnd;
  return kRequestBytes;
}

const char *parseResultName(ParseResult r) {
  switch (r) {
    case ParseResult::kOk:            return "ok";
    case ParseResult::kBadLength:     return "bad length";
    case ParseResult::kBadEnd:        return "missing 0x77 terminator";
    case ParseResult::kBadChecksum:   return "checksum mismatch";
    case ParseResult::kWrongRegister: return "reply for a different register";
    case ParseResult::kBmsError:      return "BMS returned status 0x80";
  }
  return "?";
}

ParseResult parseResponse(const uint8_t *buf, size_t n, uint8_t expectReg, Frame &out) {
  if (n < 7 || buf[0] != kStart) return ParseResult::kBadLength;
  out.reg    = buf[1];
  out.status = buf[2];
  out.len    = buf[3];
  out.data   = buf + 4;
  if (n != static_cast<size_t>(out.len) + 7) return ParseResult::kBadLength;
  if (buf[6 + out.len] != kEnd) return ParseResult::kBadEnd;
  const uint16_t got = static_cast<uint16_t>((uint16_t(buf[4 + out.len]) << 8) | buf[5 + out.len]);
  if (checksum(out.status, out.len, out.data) != got) return ParseResult::kBadChecksum;
  if (out.reg != expectReg) return ParseResult::kWrongRegister;
  if (out.status != 0x00) return ParseResult::kBmsError;
  return ParseResult::kOk;
}

bool decodeBasic(const Frame &f, BmsSnapshot &s) {
  if (f.len < 23) return false;
  const uint8_t *d = f.data;

  s.packVolts   = be16(d + 0) * 0.01f;                        // 10 mV
  s.currentAmps = static_cast<int16_t>(be16(d + 2)) * 0.01f;  // 10 mA, + = charge
  s.remainingAh = be16(d + 4) * 0.01f;                        // 10 mAh
  s.fullAh      = be16(d + 6) * 0.01f;
  s.cycles      = be16(d + 8);

  const uint16_t date = be16(d + 10);                         // (Y-2000)<<9 | M<<5 | D
  s.mfgYear  = static_cast<uint16_t>(2000 + (date >> 9));
  s.mfgMonth = static_cast<uint8_t>((date >> 5) & 0x0F);
  s.mfgDay   = static_cast<uint8_t>(date & 0x1F);

  s.balanceBits = static_cast<uint32_t>(be16(d + 12)) |
                  (static_cast<uint32_t>(be16(d + 14)) << 16);
  s.protection      = be16(d + 16);
  s.softwareVersion = d[18];
  s.socPercent      = d[19];
  s.chargeMosOn     = (d[20] & 0x01) != 0;
  s.dischargeMosOn  = (d[20] & 0x02) != 0;
  s.cellCount       = d[21];

  uint8_t n = d[22];
  if (n > kMaxNtc) n = kMaxNtc;
  if (23 + 2 * n > f.len) n = static_cast<uint8_t>((f.len - 23) / 2);
  for (uint8_t i = 0; i < n; ++i)                             // 0.1 K, offset 2731
    s.ntcCelsius[i] = (static_cast<int>(be16(d + 23 + 2 * i)) - 2731) * 0.1f;
  s.ntcCount = n;

  // Register 0x03 delivers all of these in one reply.
  s.fields |= kFieldPack | kFieldCapacity | kFieldProtect | kFieldMos | kFieldConfig | kFieldTemps;
  s.source = Source::kUart;
  s.valid  = true;
  return true;
}

bool decodeCells(const Frame &f, BmsSnapshot &s) {
  uint8_t n = static_cast<uint8_t>(f.len / 2);
  if (n == 0) return false;
  if (n > kMaxCells) n = kMaxCells;
  for (uint8_t i = 0; i < n; ++i) {
    s.cellMilliVolts[i] = be16(f.data + 2 * i);
    s.cellsReadMask |= (1u << i);
  }
  s.fields |= kFieldCells;
  if (s.cellCount == 0) s.cellCount = n;
  return true;
}

// =================================== CAN ====================================

uint16_t crc16Modbus(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

bool canCrcOk(const uint8_t *data, uint8_t dlc) {
  if (dlc < 3) return true;
  const uint8_t n = static_cast<uint8_t>(dlc - 2);
  return crc16Modbus(data, n) == static_cast<uint16_t>((uint16_t(data[n]) << 8) | data[n + 1]);
}

uint8_t canCellFrameCount(uint8_t cellCount) {
  return static_cast<uint8_t>((cellCount + 2) / 3);
}

bool decodeCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc, BmsSnapshot &s) {
  if (id < kCanPack || id > kCanCellLast) return false;
  if (dlc < 2) return id >= kCanCellFirst;  // empty frame = no cells in that range
  const uint8_t  payload = static_cast<uint8_t>(dlc - 2);
  const uint8_t *d       = data;

  switch (id) {
    case kCanPack:
      if (payload < 6) return false;
      s.packVolts   = be16(d + 0) * 0.01f;
      s.currentAmps = static_cast<int16_t>(be16(d + 2)) * 0.01f;
      s.remainingAh = be16(d + 4) * 0.01f;
      s.fields |= kFieldPack;
      break;

    case kCanCapacity:
      if (payload < 6) return false;
      s.fullAh     = be16(d + 0) * 0.01f;
      s.cycles     = be16(d + 2);
      s.socPercent = static_cast<uint8_t>(be16(d + 4));
      s.fields |= kFieldCapacity;
      break;

    case kCanBalProt:
      if (payload < 6) return false;
      s.balanceBits = static_cast<uint32_t>(be16(d + 0)) |
                      (static_cast<uint32_t>(be16(d + 2)) << 16);
      s.protection = be16(d + 4);
      s.fields |= kFieldProtect;
      break;

    case kCanMosDate: {
      if (payload < 6) return false;
      const uint16_t mos = be16(d + 0);
      s.chargeMosOn    = (mos & 0x01) != 0;
      s.dischargeMosOn = (mos & 0x02) != 0;
      const uint16_t date = be16(d + 2);
      s.mfgYear         = static_cast<uint16_t>(2000 + (date >> 9));
      s.mfgMonth        = static_cast<uint8_t>((date >> 5) & 0x0F);
      s.mfgDay          = static_cast<uint8_t>(date & 0x1F);
      s.softwareVersion = be16(d + 4);
      s.fields |= kFieldMos;
      break;
    }

    case kCanConfig:
      if (payload < 2) return false;
      s.cellCount = d[0];
      s.ntcCount  = d[1];
      s.fields |= kFieldConfig;
      break;

    case kCanNtc1:
    case kCanNtc4: {
      const uint8_t base = (id == kCanNtc1) ? 0 : 3;
      // Short frames are normal: with five NTCs, 0x106 arrives as DLC 6.
      for (uint8_t i = 0; i < 3; ++i) {
        if (2 * i + 1 >= payload) break;
        if (base + i >= kMaxNtc) break;
        s.ntcCelsius[base + i] = (static_cast<int>(be16(d + 2 * i)) - 2731) * 0.1f;
        s.fields |= kFieldTemps;
      }
      break;
    }

    default: {  // 0x107 .. 0x110, three cells each
      const uint8_t base = static_cast<uint8_t>((id - kCanCellFirst) * 3);
      for (uint8_t i = 0; i < 3; ++i) {
        if (2 * i + 1 >= payload) break;
        if (base + i >= kMaxCells) break;
        s.cellMilliVolts[base + i] = be16(d + 2 * i);
        s.cellsReadMask |= (1u << (base + i));
        s.fields |= kFieldCells;
      }
      break;
    }
  }
  s.source = Source::kCan;
  return true;
}

}  // namespace jbd
